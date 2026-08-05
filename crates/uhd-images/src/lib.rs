//! Content-addressed FPGA and firmware images for UHD devices.
//!
//! Objects live at `objects/<sha256>`; names, releases, and compatibility
//! versions are metadata and never determine storage identity.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt;
use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};

/// Cache override. Otherwise `$XDG_CACHE_HOME/uhd/images` or
/// `$HOME/.cache/uhd/images` is used.
pub const CACHE_ENV: &str = "UHD_IMAGES_CACHE_DIR";
/// Lockfile override. Otherwise `uhd-images.lock` in the current directory.
pub const LOCK_ENV: &str = "UHD_IMAGES_LOCKFILE";
/// The built-in, official manifest.
pub const BUILTIN_MANIFEST: &str = include_str!("../images.toml");

/// Library error.
#[derive(Debug)]
pub enum Error {
    /// File-system or stream error.
    Io(io::Error),
    /// Invalid manifest or lockfile.
    Format(String),
    /// Download support was disabled or a request failed.
    Download(String),
    /// Data did not match its declared identity.
    Integrity(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::Format(e) | Self::Download(e) | Self::Integrity(e) => f.write_str(e),
        }
    }
}
impl std::error::Error for Error {}
impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}
/// Crate result.
pub type Result<T> = std::result::Result<T, Error>;

/// One manifest mapping.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
pub struct Image {
    /// Device family in lowercase.
    pub device: String,
    /// `firmware` or `fpga`.
    pub kind: String,
    /// FPGA compatibility major.
    pub fpga_compat_major: u16,
    /// FPGA compatibility minor.
    pub fpga_compat_minor: u16,
    /// UHD releases that shipped this exact image.
    #[serde(default)]
    pub release: Vec<String>,
    /// File name inside the official archive.
    pub filename: String,
    /// SHA-256 of the extracted file.
    pub sha256: String,
    /// Exact extracted size.
    pub size: u64,
    /// Official archive URL.
    pub source_url: String,
    /// SHA-256 of the official archive.
    pub archive_sha256: String,
}

/// TOML image manifest.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Manifest {
    /// Format version, currently one.
    pub manifest_version: u32,
    /// Image mappings.
    #[serde(rename = "image")]
    pub images: Vec<Image>,
}

impl Manifest {
    /// Parse and validate TOML.
    pub fn parse(text: &str) -> Result<Self> {
        let value: Self = toml::from_str(text).map_err(|e| Error::Format(e.to_string()))?;
        if value.manifest_version != 1 {
            return Err(Error::Format("unsupported manifest_version".into()));
        }
        for image in &value.images {
            validate_image(image)?;
        }
        Ok(value)
    }
    /// Load the manifest compiled into this crate.
    pub fn builtin() -> Result<Self> {
        Self::parse(BUILTIN_MANIFEST)
    }
    /// Select the newest mapping for each requested device and kind.
    #[must_use]
    pub fn select(&self, devices: &[String], release: Option<&str>) -> Vec<Image> {
        let wanted: BTreeSet<_> = devices.iter().map(|s| s.to_ascii_lowercase()).collect();
        let mut selected = Vec::new();
        for device in wanted {
            for kind in ["firmware", "fpga"] {
                if let Some(image) = self.images.iter().rev().find(|i| {
                    i.device.eq_ignore_ascii_case(&device)
                        && i.kind == kind
                        && release.is_none_or(|r| i.release.iter().any(|v| v == r))
                }) {
                    selected.push(image.clone());
                }
            }
        }
        selected
    }
}

fn validate_image(i: &Image) -> Result<()> {
    if i.filename.contains('/')
        || i.filename.contains('\\')
        || i.filename == "."
        || i.filename == ".."
    {
        return Err(Error::Format(format!("unsafe filename {}", i.filename)));
    }
    for hash in [&i.sha256, &i.archive_sha256] {
        if hash.len() != 64 || !hash.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(Error::Format(format!("invalid SHA-256 {hash}")));
        }
    }
    Ok(())
}

/// A content-addressed cache.
#[derive(Clone, Debug)]
pub struct Cache {
    root: PathBuf,
}

impl Cache {
    /// Open a cache at an explicit path.
    #[must_use]
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self { root: root.into() }
    }
    /// Resolve the environment override or XDG user cache directory.
    pub fn discover() -> Result<Self> {
        if let Some(path) = std::env::var_os(CACHE_ENV).filter(|p| !p.is_empty()) {
            return Ok(Self::new(path));
        }
        let base = std::env::var_os("XDG_CACHE_HOME")
            .filter(|p| !p.is_empty())
            .map(PathBuf::from)
            .or_else(|| {
                std::env::var_os("HOME")
                    .filter(|p| !p.is_empty())
                    .map(|p| PathBuf::from(p).join(".cache"))
            })
            .ok_or_else(|| Error::Format(format!("set {CACHE_ENV}, XDG_CACHE_HOME, or HOME")))?;
        Ok(Self::new(base.join("uhd/images")))
    }
    /// Cache root.
    #[must_use]
    pub fn root(&self) -> &Path {
        &self.root
    }
    /// Path for an object's hash.
    #[must_use]
    pub fn object_path(&self, hash: &str) -> PathBuf {
        self.root.join("objects").join(hash)
    }
    /// Verify one cached object completely.
    pub fn verify(&self, image: &Image) -> Result<()> {
        let path = self.object_path(&image.sha256);
        let file =
            File::open(&path).map_err(|e| Error::Integrity(format!("{}: {e}", path.display())))?;
        verify_reader(file, image.size, &image.sha256)
            .map_err(|e| Error::Integrity(format!("{}: {e}", path.display())))
    }
    /// Verify all selected objects.
    pub fn verify_all(&self, images: &[Image]) -> Result<()> {
        for image in images {
            self.verify(image)?;
        }
        Ok(())
    }

    /// Fetch missing images, verifying archive and payload before installation.
    #[cfg(feature = "download")]
    pub fn sync(&self, images: &[Image], mut progress: impl FnMut(&str)) -> Result<Vec<PathBuf>> {
        fs::create_dir_all(self.root.join("objects"))?;
        let mut paths = Vec::new();
        for image in images {
            let dest = self.object_path(&image.sha256);
            if self.verify(image).is_ok() {
                progress(&format!("present {}", image.filename));
                paths.push(dest);
                continue;
            }
            progress(&format!("fetching {}", image.source_url));
            let mut response = ureq::get(&image.source_url)
                .header(
                    "User-Agent",
                    concat!("uhd-images/", env!("CARGO_PKG_VERSION")),
                )
                .call()
                .map_err(|e| Error::Download(format!("{}: {e}", image.source_url)))?;
            let archive = response
                .body_mut()
                .with_config()
                .limit(256 * 1024 * 1024)
                .read_to_vec()
                .map_err(|e| Error::Download(format!("{}: {e}", image.source_url)))?;
            verify_bytes(&archive, archive.len() as u64, &image.archive_sha256)?;
            let mut zip = zip::ZipArchive::new(std::io::Cursor::new(archive))
                .map_err(|e| Error::Format(e.to_string()))?;
            let mut member = zip
                .by_name(&image.filename)
                .map_err(|e| Error::Format(format!("{}: {e}", image.filename)))?;
            let mut payload = Vec::new();
            member.read_to_end(&mut payload)?;
            verify_bytes(&payload, image.size, &image.sha256)?;
            atomic_write(&dest, &payload)?;
            paths.push(dest);
        }
        Ok(paths)
    }

    /// Export selected verified objects and a lockfile as an uncompressed tar.
    pub fn export(&self, images: &[Image], bundle: &Path) -> Result<()> {
        self.verify_all(images)?;
        let lock = Lockfile::from_images(images);
        let lock_text = toml::to_string_pretty(&lock).map_err(|e| Error::Format(e.to_string()))?;
        let file = File::create(bundle)?;
        let mut tar = tar::Builder::new(file);
        append_bytes(&mut tar, "uhd-images.lock", lock_text.as_bytes())?;
        for image in images {
            tar.append_path_with_name(
                self.object_path(&image.sha256),
                format!("objects/{}", image.sha256),
            )?;
        }
        tar.finish()?;
        Ok(())
    }

    /// Import an exported bundle, rejecting unlisted or corrupt objects.
    pub fn import(&self, bundle: &Path) -> Result<Lockfile> {
        let staging = self.root.join(format!(".import-{}", std::process::id()));
        if staging.exists() {
            fs::remove_dir_all(&staging)?;
        }
        fs::create_dir_all(&staging)?;
        let result = (|| {
            let mut archive = tar::Archive::new(File::open(bundle)?);
            archive.unpack(&staging)?;
            let lock = Lockfile::read(&staging.join("uhd-images.lock"))?;
            fs::create_dir_all(self.root.join("objects"))?;
            for image in &lock.images {
                validate_image(image)?;
                let source = staging.join("objects").join(&image.sha256);
                verify_reader(File::open(&source)?, image.size, &image.sha256)?;
                let bytes = fs::read(source)?;
                atomic_write(&self.object_path(&image.sha256), &bytes)?;
            }
            Ok(lock)
        })();
        let _ = fs::remove_dir_all(staging);
        result
    }
}

/// Reproducible set of exact image identities.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Lockfile {
    /// Format version.
    pub lock_version: u32,
    /// Exact images.
    #[serde(rename = "image")]
    pub images: Vec<Image>,
}
impl Lockfile {
    /// Construct from a resolved selection.
    #[must_use]
    pub fn from_images(images: &[Image]) -> Self {
        Self {
            lock_version: 1,
            images: images.to_vec(),
        }
    }
    /// Read TOML.
    pub fn read(path: &Path) -> Result<Self> {
        let value: Self =
            toml::from_str(&fs::read_to_string(path)?).map_err(|e| Error::Format(e.to_string()))?;
        if value.lock_version != 1 {
            return Err(Error::Format("unsupported lock_version".into()));
        }
        Ok(value)
    }
    /// Write TOML atomically.
    pub fn write(&self, path: &Path) -> Result<()> {
        let text = toml::to_string_pretty(self).map_err(|e| Error::Format(e.to_string()))?;
        atomic_write(path, text.as_bytes())
    }
}

fn verify_reader(mut reader: impl Read, size: u64, hash: &str) -> Result<()> {
    let mut bytes = Vec::new();
    reader.read_to_end(&mut bytes)?;
    verify_bytes(&bytes, size, hash)
}
fn verify_bytes(bytes: &[u8], size: u64, hash: &str) -> Result<()> {
    if bytes.len() as u64 != size {
        return Err(Error::Integrity(format!(
            "size mismatch: expected {size}, got {}",
            bytes.len()
        )));
    }
    let found = Sha256::digest(bytes)
        .iter()
        .fold(String::new(), |mut out, byte| {
            use std::fmt::Write as _;
            let _ = write!(out, "{byte:02x}");
            out
        });
    if found != hash {
        return Err(Error::Integrity(format!(
            "SHA-256 mismatch: expected {hash}, got {found}"
        )));
    }
    Ok(())
}
fn atomic_write(path: &Path, bytes: &[u8]) -> Result<()> {
    let parent = path
        .parent()
        .ok_or_else(|| Error::Format("path has no parent".into()))?;
    fs::create_dir_all(parent)?;
    let temp = parent.join(format!(
        ".tmp-{}-{}",
        std::process::id(),
        path.file_name().unwrap_or_default().to_string_lossy()
    ));
    {
        let mut file = File::create(&temp)?;
        file.write_all(bytes)?;
        file.sync_all()?;
    }
    fs::rename(temp, path)?;
    Ok(())
}
fn append_bytes(tar: &mut tar::Builder<File>, name: &str, bytes: &[u8]) -> Result<()> {
    let mut header = tar::Header::new_gnu();
    header.set_size(bytes.len() as u64);
    header.set_mode(0o644);
    header.set_cksum();
    tar.append_data(&mut header, name, bytes)?;
    Ok(())
}

/// Bitstream compiled with `embedded-images` and `UHD_EMBEDDED_IMAGE`.
#[cfg(feature = "embedded-images")]
#[must_use]
pub fn embedded_image() -> Option<&'static [u8]> {
    include!(concat!(env!("OUT_DIR"), "/embedded.rs"))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn builtin_is_valid_and_has_release_history() {
        let m = Manifest::builtin().unwrap();
        assert_eq!(m.images.len(), 3);
        assert_eq!(m.select(&["b205mini".into()], Some("v4.6.0.0")).len(), 2);
        assert_eq!(m.select(&["b205mini".into()], Some("v4.10.0.0")).len(), 2);
    }
    #[test]
    fn object_paths_are_hash_keyed() {
        let c = Cache::new("cache");
        assert_eq!(c.object_path("abc"), Path::new("cache/objects/abc"));
    }
}
