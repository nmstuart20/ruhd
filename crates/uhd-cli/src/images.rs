//! `ruhd images download` — fetching the UHD images package.
//!
//! The rough equivalent of `uhd_images_downloader.py`, narrowed to the B2xx
//! family. A B2xx has no flash: the FX3 firmware and the FPGA bitstream are
//! pushed over USB on every plug-in, so without these files a board enumerates
//! and does nothing else. That makes "go and get them" a first-class command
//! rather than a setup note, which is why [`uhd_transport::images`]'s
//! not-found error names this command.
//!
//! # The manifest
//!
//! UHD's downloader is a template: CMake substitutes the contents of
//! `images/manifest.txt` into `_MANIFEST_CONTENTS` at build time, so the
//! version of the images a given UHD fetches is pinned by the source tree it
//! was built from. [`MANIFEST`] is the same idea done at compile time here —
//! the B2xx rows of `reference/uhd/images/manifest.txt`, transcribed, pinned to
//! the same UHD release the reference checkout is pinned to.
//!
//! The consequence worth knowing: bumping the reference submodule and bumping
//! this table are the same act. The firmware compatibility check in
//! [`uhd_transport::usb::fx3`] is what catches it if they drift, because it
//! compares what the *device* reports against what this driver was written for.
//!
//! # Where files go
//!
//! Into [`uhd_transport::images::default_download_dir`] unless told otherwise —
//! a user-owned directory that is already first on the image search path. UHD's
//! downloader writes into the install prefix instead and is therefore usually a
//! `sudo` operation; this one never needs root, and a freshly downloaded image
//! shadows a stale system-wide one rather than fighting it.

use std::fs;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};
use tracing::debug;
use uhd_core::{Error, Result};
use uhd_transport::images;

/// Where the archives are served from.
///
/// `_DEFAULT_BASE_URL` in `uhd_images_downloader.py.in`.
pub(crate) const DEFAULT_BASE_URL: &str = "https://files.ettus.com/binaries/cache/";

/// Environment variable overriding [`DEFAULT_BASE_URL`], as UHD's downloader
/// also honours.
pub(crate) const BASE_URL_VAR: &str = "UHD_IMAGES_BASE_URL";

/// The inventory file UHD's downloader keeps beside the images.
pub(crate) const INVENTORY_FILE: &str = "inventory.json";

/// The UHD release [`MANIFEST`] was transcribed from.
pub(crate) const MANIFEST_UHD_VERSION: &str = "4.10.0.0";

/// One row of the images manifest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Target {
    /// Manifest target name, e.g. `b2xx_common_fw_default`.
    pub name: &'static str,
    /// The images-repository revision this archive was built from. This is the
    /// version identity: it is what goes in the inventory and what an
    /// already-installed archive is compared against.
    pub repo_hash: &'static str,
    /// Path under the base URL.
    pub url_path: &'static str,
    /// SHA-256 of the archive, lowercase hex. `"0"` in the manifest means the
    /// archive is not checksummed upstream; none of the B2xx rows are like
    /// that, but the format allows it.
    pub sha256: &'static str,
}

impl Target {
    /// The archive's file name.
    fn filename(&self) -> &str {
        self.url_path.rsplit('/').next().unwrap_or(self.url_path)
    }

    /// The full URL to fetch, given a base.
    fn url(&self, base_url: &str) -> String {
        let base = base_url.strip_suffix('/').unwrap_or(base_url);
        format!("{base}/{}", self.url_path)
    }
}

/// The B2xx rows of `images/manifest.txt`, at UHD [`MANIFEST_UHD_VERSION`].
///
/// The firmware row is shared by the whole family — one FX3 image drives every
/// B2xx — and is [`FIRMWARE_TARGET`]. The FPGA rows are per-product; the
/// B206mini deliberately points at the B205mini archive, exactly as the
/// manifest does.
pub(crate) const MANIFEST: [Target; 6] = [
    Target {
        name: "b2xx_common_fw_default",
        repo_hash: "uhd-7f7d016",
        url_path: "b2xx/uhd-7f7d016/b2xx_common_fw_default-g7f7d016.zip",
        sha256: "22ac0e54cc90d53d8494d481439447a0c89b9bbaced6fabe24df260eda3ce3a3",
    },
    Target {
        name: "b2xx_b200_fpga_default",
        repo_hash: "uhd-c37b318",
        url_path: "b2xx/uhd-c37b318/b2xx_b200_fpga_default-gc37b318.zip",
        sha256: "35542b8a2b6a6dfd3aa155c35364af85c2599cfb692b0cf94013c5342c945448",
    },
    Target {
        name: "b2xx_b210_fpga_default",
        repo_hash: "uhd-c37b318",
        url_path: "b2xx/uhd-c37b318/b2xx_b210_fpga_default-gc37b318.zip",
        sha256: "ab1cdbe93090fdcf590a79cbe938749f8a5fbd7b45422b7833b8c841aa2106b4",
    },
    Target {
        name: "b2xx_b200mini_fpga_default",
        repo_hash: "uhd-c37b318",
        url_path: "b2xx/uhd-c37b318/b2xx_b200mini_fpga_default-gc37b318.zip",
        sha256: "c1865c6e574cf453a688c6bfb8fe5e3646cd53458ae403cf11d3eeb7dc6a9918",
    },
    Target {
        name: "b2xx_b205mini_fpga_default",
        repo_hash: "uhd-c37b318",
        url_path: "b2xx/uhd-c37b318/b2xx_b205mini_fpga_default-gc37b318.zip",
        sha256: "dce96120e3512d6e21bb061811c4021cbc90ac4d787248ad821337b7389ff3c4",
    },
    Target {
        name: "b2xx_b206mini_fpga_default",
        repo_hash: "uhd-c37b318",
        url_path: "b2xx/uhd-c37b318/b2xx_b205mini_fpga_default-gc37b318.zip",
        sha256: "dce96120e3512d6e21bb061811c4021cbc90ac4d787248ad821337b7389ff3c4",
    },
];

/// The manifest target holding `usrp_b200_fw.hex`.
///
/// Named because the firmware is the one image without which nothing at all
/// works, so it is what the version check in `ruhd init` is about.
pub(crate) const FIRMWARE_TARGET: &Target = &MANIFEST[0];

/// Looks up a manifest row by target name.
#[must_use]
pub(crate) fn target(name: &str) -> Option<&'static Target> {
    MANIFEST.iter().find(|t| t.name == name)
}

/// The base URL to fetch from: `--base-url`, then [`BASE_URL_VAR`], then
/// [`DEFAULT_BASE_URL`].
#[must_use]
pub(crate) fn base_url(override_url: Option<&str>) -> String {
    override_url
        .map(str::to_owned)
        .or_else(|| std::env::var(BASE_URL_VAR).ok().filter(|s| !s.is_empty()))
        .unwrap_or_else(|| DEFAULT_BASE_URL.to_owned())
}

/// The `inventory.json` UHD's downloader keeps beside the images.
///
/// Only the fields this driver reads or writes are modelled. The file is
/// treated as shared property: an entry written here is one
/// `uhd_images_downloader` will understand and skip, and vice versa, so the two
/// downloaders can be pointed at the same directory without either re-fetching
/// what the other installed.
#[derive(Debug, Default)]
pub(crate) struct Inventory {
    path: PathBuf,
    root: serde_json::Map<String, serde_json::Value>,
}

impl Inventory {
    /// Reads the inventory beside `dir`, or an empty one.
    ///
    /// A corrupt or unreadable inventory is treated as empty rather than as an
    /// error — same as UHD, and for the same reason: the file is a cache of
    /// what is already installed, and the worst consequence of losing it is
    /// downloading something twice.
    #[must_use]
    pub(crate) fn load(dir: &Path) -> Self {
        let path = dir.join(INVENTORY_FILE);
        let root = fs::read_to_string(&path)
            .ok()
            .and_then(|text| serde_json::from_str::<serde_json::Value>(&text).ok())
            .and_then(|value| match value {
                serde_json::Value::Object(map) => Some(map),
                _ => None,
            })
            .unwrap_or_else(|| {
                debug!(path = %path.display(), "no usable inventory; assuming empty");
                serde_json::Map::new()
            });
        Self { path, root }
    }

    /// The images-repository revision recorded for `target`, if any.
    #[must_use]
    pub(crate) fn repo_hash(&self, target: &str) -> Option<&str> {
        self.root.get(target)?.get("repo_hash")?.as_str()
    }

    /// Records a freshly installed archive.
    fn insert(&mut self, target: &Target, contents: Vec<String>) {
        self.root.insert(
            target.name.to_owned(),
            serde_json::json!({
                "repo_hash": target.repo_hash,
                "filename": target.filename(),
                "contents": contents,
            }),
        );
    }

    /// Writes the inventory back out.
    fn save(&self) -> Result<()> {
        let text = serde_json::Value::Object(self.root.clone()).to_string();
        fs::write(&self.path, text).map_err(|e| {
            Error::Io(std::io::Error::new(
                e.kind(),
                format!("cannot write {}: {e}", self.path.display()),
            ))
        })
    }
}

/// What a target's installation status is, for reporting before doing anything.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub(crate) enum Status {
    /// Installed, and at the revision [`MANIFEST`] pins.
    UpToDate,
    /// Installed, but at a different revision. Holds what the inventory says.
    Stale(String),
    /// No inventory entry for this target in any images directory.
    Missing,
}

/// Reports whether `target` is installed at the pinned revision.
///
/// Every directory on the image search path is consulted, in order, and the
/// first that has an inventory entry for this target answers — which mirrors
/// how [`images::find_image`] resolves the file itself.
///
/// [`Status::Missing`] covers both "never installed" and "installed by
/// something that kept no inventory", so it is a weaker signal than it looks:
/// a system UHD package that shipped images without an `inventory.json` reads
/// as missing even though the file is right there. That is why `ruhd init`
/// treats a bad status as a warning and lets the firmware compatibility check
/// on the device have the final word.
#[must_use]
pub(crate) fn status(target: &Target) -> Status {
    for dir in images::search_paths() {
        match Inventory::load(&dir).repo_hash(target.name) {
            Some(hash) if hash == target.repo_hash => return Status::UpToDate,
            Some(hash) => return Status::Stale(hash.to_owned()),
            None => {}
        }
    }
    Status::Missing
}

/// How a [`download`] went for one target.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub(crate) enum Fetched {
    /// Already installed at the pinned revision; nothing was fetched.
    Skipped,
    /// Downloaded and extracted. Holds the archive size in bytes and the file
    /// names that were written.
    Installed {
        /// Size of the downloaded archive.
        bytes: u64,
        /// Files extracted from it.
        files: Vec<String>,
    },
}

/// Downloads and installs a set of manifest targets.
///
/// Returns one outcome per target, in the order given. `force` re-fetches even
/// targets the inventory already lists at the pinned revision.
///
/// # Errors
///
/// [`Error::Io`] if the destination cannot be written; [`Error::Transport`] if
/// a download fails or the archive's SHA-256 does not match the manifest.
/// A checksum mismatch aborts before anything is extracted.
pub(crate) fn download(
    targets: &[&'static Target],
    dir: &Path,
    base_url: &str,
    force: bool,
    mut progress: impl FnMut(&str),
) -> Result<Vec<Fetched>> {
    fs::create_dir_all(dir).map_err(|e| {
        Error::Io(std::io::Error::new(
            e.kind(),
            format!("cannot create {}: {e}", dir.display()),
        ))
    })?;

    let mut inventory = Inventory::load(dir);
    let mut outcomes = Vec::with_capacity(targets.len());

    for target in targets {
        if !force && inventory.repo_hash(target.name) == Some(target.repo_hash) {
            progress(&format!(
                "{} is already at {}",
                target.name, target.repo_hash
            ));
            outcomes.push(Fetched::Skipped);
            continue;
        }

        let url = target.url(base_url);
        progress(&format!("fetching {}", url));

        let archive = fetch(&url)?;
        verify(target, &archive, &url)?;

        let files = extract(&archive, dir)?;
        progress(&format!(
            "  extracted {} file(s) into {}",
            files.len(),
            dir.display()
        ));

        inventory.insert(target, files.clone());
        outcomes.push(Fetched::Installed {
            bytes: archive.len() as u64,
            files,
        });
    }

    inventory.save()?;
    Ok(outcomes)
}

/// Largest archive this will accept, as a guard against a redirect to
/// something enormous. The biggest B2xx archive is under 5 MB; the limit is set
/// far above that so it never fires in normal use.
const MAX_ARCHIVE_BYTES: u64 = 256 * 1024 * 1024;

/// Fetches one archive into memory.
///
/// In memory rather than to a temporary file because these are a few megabytes
/// each and the zip reader wants `Seek` anyway — and because it means a failed
/// download leaves nothing behind to clean up.
fn fetch(url: &str) -> Result<Vec<u8>> {
    let mut response = ureq::get(url)
        .header("User-Agent", concat!("ruhd/", env!("CARGO_PKG_VERSION")))
        .call()
        .map_err(|e| Error::Transport(format!("cannot fetch {url}: {e}")))?;

    if !response.status().is_success() {
        return Err(Error::Transport(format!(
            "cannot fetch {url}: server returned {}",
            response.status()
        )));
    }

    response
        .body_mut()
        .with_config()
        .limit(MAX_ARCHIVE_BYTES)
        .read_to_vec()
        .map_err(|e| Error::Transport(format!("error reading {url}: {e}")))
}

/// Checks an archive against the manifest's SHA-256.
fn verify(target: &Target, archive: &[u8], url: &str) -> Result<()> {
    // "0" is the manifest's marker for an unchecksummed archive.
    if target.sha256 == "0" {
        debug!(target = target.name, "manifest carries no checksum");
        return Ok(());
    }

    let digest = Sha256::digest(archive);
    let found = digest.iter().fold(String::new(), |mut s, b| {
        use std::fmt::Write as _;
        let _ = write!(s, "{b:02x}");
        s
    });

    if found != target.sha256 {
        return Err(Error::Transport(format!(
            "checksum mismatch for {}\n  url:      {url}\n  expected: {}\n  got:      {found}\n\
             Nothing was installed. A mismatch is either a corrupted download or a base \
             URL serving different images than the manifest pins.",
            target.name, target.sha256,
        )));
    }
    Ok(())
}

/// Extracts an archive into `dir`, returning the file names written.
///
/// Entry paths are taken through `enclosed_name`, which rejects absolute paths
/// and `..` components — an archive that tried to write outside `dir` is an
/// error, not a surprise.
fn extract(archive: &[u8], dir: &Path) -> Result<Vec<String>> {
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(archive))
        .map_err(|e| Error::Transport(format!("cannot read the downloaded archive: {e}")))?;

    let mut written = Vec::new();
    for i in 0..zip.len() {
        let mut entry = zip
            .by_index(i)
            .map_err(|e| Error::Transport(format!("cannot read archive entry {i}: {e}")))?;

        let Some(relative) = entry.enclosed_name() else {
            return Err(Error::Transport(format!(
                "archive entry {:?} escapes the destination directory",
                entry.name()
            )));
        };
        let path = dir.join(&relative);

        if entry.is_dir() {
            fs::create_dir_all(&path).map_err(|e| io_error(&path, e))?;
            continue;
        }
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| io_error(parent, e))?;
        }

        let mut out = fs::File::create(&path).map_err(|e| io_error(&path, e))?;
        let mut buf = [0u8; 64 * 1024];
        loop {
            let n = entry.read(&mut buf).map_err(|e| {
                Error::Transport(format!("error decompressing {}: {e}", relative.display()))
            })?;
            if n == 0 {
                break;
            }
            out.write_all(&buf[..n]).map_err(|e| io_error(&path, e))?;
        }

        written.push(relative.to_string_lossy().into_owned());
    }

    Ok(written)
}

fn io_error(path: &Path, e: std::io::Error) -> Error {
    Error::Io(std::io::Error::new(
        e.kind(),
        format!("{}: {e}", path.display()),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_target_has_a_plausible_manifest_row() {
        for t in &MANIFEST {
            assert!(t.name.starts_with("b2xx_"), "{}", t.name);
            assert!(t.url_path.starts_with("b2xx/"), "{}", t.url_path);
            assert!(t.url_path.ends_with(".zip"), "{}", t.url_path);
            // The manifest embeds the repo hash in the path, so a row where
            // the two disagree is a transcription slip.
            assert!(
                t.url_path.contains(t.repo_hash),
                "{} does not contain {}",
                t.url_path,
                t.repo_hash
            );
            assert!(
                t.sha256 == "0" || t.sha256.len() == 64,
                "{} has a {}-char checksum",
                t.name,
                t.sha256.len()
            );
            assert!(t.sha256.chars().all(|c| c.is_ascii_hexdigit()));
        }
    }

    #[test]
    fn the_firmware_target_is_the_one_that_carries_the_hex_file() {
        assert_eq!(FIRMWARE_TARGET.name, "b2xx_common_fw_default");
        assert_eq!(target("b2xx_common_fw_default"), Some(FIRMWARE_TARGET));
        assert_eq!(target("nonsense"), None);
    }

    #[test]
    fn the_b206mini_shares_the_b205minis_bitstream() {
        // As the manifest does. Worth pinning: it looks like a copy-paste bug
        // until you know the two boards take the same image.
        let b205 = target("b2xx_b205mini_fpga_default").unwrap();
        let b206 = target("b2xx_b206mini_fpga_default").unwrap();
        assert_eq!(b205.url_path, b206.url_path);
        assert_eq!(b205.sha256, b206.sha256);
    }

    #[test]
    fn urls_join_without_doubling_the_slash() {
        let t = FIRMWARE_TARGET;
        assert_eq!(
            t.url("https://example.com/cache/"),
            format!("https://example.com/cache/{}", t.url_path)
        );
        assert_eq!(
            t.url("https://example.com/cache"),
            format!("https://example.com/cache/{}", t.url_path)
        );
    }

    #[test]
    fn filenames_come_off_the_end_of_the_url_path() {
        assert_eq!(
            FIRMWARE_TARGET.filename(),
            "b2xx_common_fw_default-g7f7d016.zip"
        );
    }

    #[test]
    fn the_base_url_override_order_is_flag_then_env_then_default() {
        assert_eq!(
            base_url(Some("https://mirror.example/")),
            "https://mirror.example/"
        );
        // Not touching the process environment here: the env arm is exercised
        // by the default arm being reached only when the var is unset, and
        // mutating env vars in a threaded test runner is unsound.
        if std::env::var_os(BASE_URL_VAR).is_none() {
            assert_eq!(base_url(None), DEFAULT_BASE_URL);
        }
    }

    #[test]
    fn a_missing_inventory_reads_as_empty_rather_than_failing() {
        let inventory = Inventory::load(Path::new("/nonexistent/ruhd/images"));
        assert_eq!(inventory.repo_hash("b2xx_common_fw_default"), None);
    }

    #[test]
    fn a_corrupt_inventory_reads_as_empty_too() {
        let dir = std::env::temp_dir().join(format!("ruhd-inv-test-{}", std::process::id()));
        fs::create_dir_all(&dir).expect("temp dir");
        fs::write(dir.join(INVENTORY_FILE), "{not json at all").expect("write");
        assert_eq!(Inventory::load(&dir).repo_hash("anything"), None);
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_inventory_round_trips_through_the_uhd_format() {
        let dir = std::env::temp_dir().join(format!("ruhd-inv-rt-{}", std::process::id()));
        fs::create_dir_all(&dir).expect("temp dir");

        let mut inventory = Inventory::load(&dir);
        inventory.insert(FIRMWARE_TARGET, vec!["usrp_b200_fw.hex".into()]);
        inventory.save().expect("save");

        // Reloaded through the same reader UHD's would see.
        let reloaded = Inventory::load(&dir);
        assert_eq!(
            reloaded.repo_hash(FIRMWARE_TARGET.name),
            Some(FIRMWARE_TARGET.repo_hash)
        );

        // And the shape is the one uhd_images_downloader.py writes.
        let text = fs::read_to_string(dir.join(INVENTORY_FILE)).expect("read");
        let value: serde_json::Value = serde_json::from_str(&text).expect("valid json");
        let entry = &value[FIRMWARE_TARGET.name];
        assert_eq!(entry["repo_hash"], FIRMWARE_TARGET.repo_hash);
        assert_eq!(entry["filename"], FIRMWARE_TARGET.filename());
        assert_eq!(entry["contents"][0], "usrp_b200_fw.hex");

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_checksum_mismatch_names_both_hashes_and_installs_nothing() {
        let e = verify(FIRMWARE_TARGET, b"not the archive", "https://example/x.zip")
            .expect_err("mismatch");
        let msg = e.to_string();
        assert!(msg.contains(FIRMWARE_TARGET.sha256), "{msg}");
        assert!(msg.contains("Nothing was installed"), "{msg}");
    }

    #[test]
    fn an_unchecksummed_manifest_row_is_accepted() {
        let unchecked = Target {
            sha256: "0",
            ..*FIRMWARE_TARGET
        };
        assert!(verify(&unchecked, b"anything", "https://example/x.zip").is_ok());
    }

    #[test]
    fn extraction_rejects_an_entry_that_escapes_the_destination() {
        // Hand-built zip with a `../` entry name. `enclosed_name` is what
        // catches it; this proves the guard is actually wired up.
        let mut buf = Vec::new();
        {
            let mut writer = zip::ZipWriter::new(std::io::Cursor::new(&mut buf));
            let options: zip::write::FileOptions<'_, ()> = zip::write::FileOptions::default();
            writer
                .start_file("../escaped.txt", options)
                .expect("start file");
            writer.write_all(b"nope").expect("write");
            writer.finish().expect("finish");
        }

        let dir = std::env::temp_dir().join(format!("ruhd-zip-test-{}", std::process::id()));
        fs::create_dir_all(&dir).expect("temp dir");
        let e = extract(&buf, &dir).expect_err("path traversal is rejected");
        assert!(e.to_string().contains("escapes"), "{e}");
        assert!(!dir.parent().unwrap().join("escaped.txt").exists());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn extraction_writes_what_the_archive_holds() {
        let mut buf = Vec::new();
        {
            let mut writer = zip::ZipWriter::new(std::io::Cursor::new(&mut buf));
            let options: zip::write::FileOptions<'_, ()> = zip::write::FileOptions::default();
            writer
                .start_file("usrp_b200_fw.hex", options)
                .expect("start");
            writer.write_all(b":00000001FF\n").expect("write");
            writer.finish().expect("finish");
        }

        let dir = std::env::temp_dir().join(format!("ruhd-zip-ok-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("temp dir");

        let files = extract(&buf, &dir).expect("extracts");
        assert_eq!(files, vec!["usrp_b200_fw.hex".to_string()]);
        assert_eq!(
            fs::read_to_string(dir.join("usrp_b200_fw.hex")).expect("read"),
            ":00000001FF\n"
        );
        let _ = fs::remove_dir_all(&dir);
    }
}
