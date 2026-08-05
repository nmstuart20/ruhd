//! Finding the UHD images package on disk.
//!
//! A B2xx has no flash. The FX3 firmware and the FPGA bitstream both live in
//! the images package and are pushed over USB on every plug-in, so "where are
//! the images?" is a question that has to be answered before a device can be
//! opened at all — and answered with a useful error when it cannot be, because
//! a missing images package is the single most common reason a brand-new USRP
//! does not work.
//!
//! Mirrors `uhd::get_images_dir` and `uhd::find_image_path` in
//! `host/lib/utils/paths.cpp`.
//!
//! ```no_run
//! use uhd_transport::{images, usb};
//!
//! # fn main() -> uhd_core::Result<()> {
//! let firmware = images::find_image(usb::B200_FW_IMAGE)?;
//! println!("loading {}", firmware.display());
//! # Ok(())
//! # }
//! ```
//!
//! # Search order
//!
//! 1. Every path in `UHD_IMAGES_DIR`, in order. It is a path *list* — colon
//!    separated on Unix, semicolon on Windows — not a single directory.
//! 2. The conventional install locations for the platform, from
//!    [`default_dirs`].
//!
//! Setting `UHD_IMAGES_DIR` therefore prepends rather than replaces, which is
//! what makes [`default_download_dir`] usable without root: `ruhd images
//! download` writes into a user-owned directory that is already on the list.
//!
//! # Divergence from UHD
//!
//! `uhd::find_image_path` resolves the images *directory* first — the first
//! candidate that exists — and then requires the file to be in that one
//! directory, failing if it is not. So a `UHD_IMAGES_DIR` holding a single
//! hand-built FPGA image hides a perfectly good system-wide images package.
//! [`find_image`] searches every candidate directory for the file it was asked
//! for and takes the first hit, which is both more useful and easier to
//! explain; [`images_dir`] keeps UHD's first-directory-wins behaviour for
//! callers that want the directory itself.

use std::path::{Path, PathBuf};

use uhd_core::{Error, Result};

/// Environment variable holding the images search path.
pub const IMAGES_DIR_VAR: &str = "UHD_IMAGES_DIR";

/// The command to run when images are missing.
///
/// UHD points at `uhd_images_downloader.py` here, via `print_utility_error`.
/// This is the same idea and the same layering compromise: the transport layer
/// naming a command from the binary above it, because the alternative is an
/// error that says what went wrong and not what to do about it.
pub const DOWNLOAD_COMMAND: &str = "ruhd images download";

/// Splits `UHD_IMAGES_DIR` into its component paths.
///
/// Empty components are dropped, matching `get_env_paths`, so a trailing
/// separator or an accidental `::` is not an error.
#[must_use]
pub fn env_dirs() -> Vec<PathBuf> {
    let Some(value) = std::env::var_os(IMAGES_DIR_VAR) else {
        return Vec::new();
    };
    std::env::split_paths(&value)
        .filter(|p| !p.as_os_str().is_empty())
        .collect()
}

/// The conventional install locations for this platform, in the order UHD's
/// own packages use them.
///
/// These stand in for UHD's compiled-in `install_prefix()`: this driver has no
/// install step to bake a prefix into, so the list is fixed and covers the
/// distribution packages, a `make install` into `/usr/local`, Homebrew, and the
/// per-user location [`default_download_dir`] writes to.
#[must_use]
pub fn default_dirs() -> Vec<PathBuf> {
    let mut dirs = Vec::new();

    if let Some(dir) = user_data_dir() {
        dirs.push(dir.join("uhd").join("images"));
    }

    #[cfg(windows)]
    {
        for var in ["ProgramFiles", "ProgramFiles(x86)"] {
            if let Some(prefix) = std::env::var_os(var) {
                dirs.push(
                    PathBuf::from(prefix)
                        .join("UHD")
                        .join("share")
                        .join("uhd")
                        .join("images"),
                );
            }
        }
    }

    #[cfg(not(windows))]
    {
        dirs.push(PathBuf::from("/usr/local/share/uhd/images"));
        dirs.push(PathBuf::from("/usr/share/uhd/images"));
        // Homebrew, Apple silicon and Intel respectively.
        dirs.push(PathBuf::from("/opt/homebrew/share/uhd/images"));
        dirs.push(PathBuf::from("/opt/local/share/uhd/images"));
    }

    dirs
}

/// Every directory [`find_image`] will look in, in order.
#[must_use]
pub fn search_paths() -> Vec<PathBuf> {
    let mut dirs = env_dirs();
    dirs.extend(default_dirs());
    dirs
}

/// The images directory, the way UHD resolves it: the first candidate that
/// exists as a directory, whether or not it holds the image you want.
///
/// Returns `None` when no candidate exists, which is the state a machine that
/// has never had UHD installed is in.
#[must_use]
pub fn images_dir() -> Option<PathBuf> {
    search_paths().into_iter().find(|p| p.is_dir())
}

/// Where `ruhd images download` writes by default.
///
/// A user-owned directory — `$XDG_DATA_HOME/uhd/images`, or the platform
/// equivalent — rather than a system prefix, so downloading images is not an
/// operation that needs root. It is on [`search_paths`] ahead of the system
/// locations, so an image downloaded here shadows a stale system one.
///
/// # Errors
///
/// [`Error::Io`] if the platform has no home directory to put it in.
pub fn default_download_dir() -> Result<PathBuf> {
    user_data_dir()
        .map(|dir| dir.join("uhd").join("images"))
        .ok_or_else(|| {
            Error::Io(std::io::Error::other(
                "cannot determine a user data directory; pass an explicit destination",
            ))
        })
}

/// `$XDG_DATA_HOME`, `%LOCALAPPDATA%`, or the usual fallback under `$HOME`.
fn user_data_dir() -> Option<PathBuf> {
    #[cfg(windows)]
    {
        std::env::var_os("LOCALAPPDATA").map(PathBuf::from)
    }

    #[cfg(not(windows))]
    {
        if let Some(dir) = std::env::var_os("XDG_DATA_HOME")
            && !dir.is_empty()
        {
            return Some(PathBuf::from(dir));
        }
        let home = PathBuf::from(std::env::var_os("HOME")?);
        if cfg!(target_os = "macos") {
            Some(home.join("Library").join("Application Support"))
        } else {
            Some(home.join(".local").join("share"))
        }
    }
}

/// Locates one image by file name.
///
/// A name that is already a path to an existing file is returned as-is, which
/// is how a `fw=` device-args override or a `--fw` flag gets to point at a
/// locally built image. Otherwise every directory in [`search_paths`] is tried
/// in order.
///
/// # Errors
///
/// [`Error::Firmware`] if the image is nowhere to be found. The message lists
/// the directories that were searched and names [`DOWNLOAD_COMMAND`], because
/// "not found" on its own sends people to a forum.
pub fn find_image(name: impl AsRef<Path>) -> Result<PathBuf> {
    let name = name.as_ref();

    if name.is_file() {
        return std::fs::canonicalize(name).or_else(|_| Ok(name.to_path_buf()));
    }

    let dirs = search_paths();
    for dir in &dirs {
        let candidate = dir.join(name);
        if candidate.is_file() {
            return Ok(candidate);
        }
    }

    Err(Error::Firmware(missing_image_message(name, &dirs)))
}

/// The "image not found" message, kept separate so its wording can be tested.
fn missing_image_message(name: &Path, searched: &[PathBuf]) -> String {
    use std::fmt::Write as _;

    let mut msg = format!("cannot find the image `{}`.\n", name.display());
    if searched.is_empty() {
        msg.push_str("No image directories were searched: neither UHD_IMAGES_DIR nor any\n");
        msg.push_str("of the default install locations is set on this platform.\n");
    } else {
        msg.push_str("Searched:\n");
        for dir in searched {
            let _ = writeln!(
                msg,
                "  {} {}",
                dir.display(),
                if dir.is_dir() { "" } else { "(does not exist)" }
            );
        }
    }
    let _ = write!(
        msg,
        "\nRun `{DOWNLOAD_COMMAND}` to fetch the images package, or set \
         {IMAGES_DIR_VAR}\nto the directory holding an existing one."
    );
    msg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_path_to_an_existing_file_is_taken_as_is() {
        // Any file will do; what is under test is that a path is not looked up
        // in the images directories at all.
        let this_file = PathBuf::from(file!());
        if !this_file.is_file() {
            eprintln!("skipping: tests are not run from the crate root");
            return;
        }
        let found = find_image(&this_file).expect("an existing path resolves");
        assert!(found.is_file());
        assert!(found.ends_with("images.rs"), "{}", found.display());
    }

    #[test]
    fn a_missing_image_says_what_to_do_about_it() {
        let e = find_image("definitely_not_an_image_ruhd.hex").expect_err("not found");
        let msg = e.to_string();
        assert!(msg.contains("definitely_not_an_image_ruhd.hex"), "{msg}");
        assert!(msg.contains(DOWNLOAD_COMMAND), "{msg}");
        assert!(msg.contains(IMAGES_DIR_VAR), "{msg}");
    }

    #[test]
    fn the_message_lists_the_directories_it_searched() {
        let dirs = vec![PathBuf::from("/nowhere/one"), PathBuf::from("/nowhere/two")];
        let msg = missing_image_message(Path::new("x.hex"), &dirs);
        assert!(msg.contains("/nowhere/one"), "{msg}");
        assert!(msg.contains("/nowhere/two"), "{msg}");
        assert!(msg.contains("does not exist"), "{msg}");
    }

    #[test]
    fn the_message_survives_having_nowhere_to_look() {
        let msg = missing_image_message(Path::new("x.hex"), &[]);
        assert!(msg.contains("No image directories were searched"), "{msg}");
        assert!(msg.contains(DOWNLOAD_COMMAND), "{msg}");
    }

    #[test]
    fn env_dirs_is_a_path_list_not_a_path() {
        // `std::env::split_paths` is the platform-correct separator, which is
        // the whole point of going through it rather than splitting on ':'.
        let joined =
            std::env::join_paths([PathBuf::from("/a"), PathBuf::from("/b")]).expect("joinable");
        let split: Vec<PathBuf> = std::env::split_paths(&joined).collect();
        assert_eq!(split, vec![PathBuf::from("/a"), PathBuf::from("/b")]);
    }

    #[test]
    fn the_user_download_dir_comes_before_the_system_ones() {
        let Ok(download) = default_download_dir() else {
            eprintln!("skipping: no home directory");
            return;
        };
        let dirs = default_dirs();
        assert_eq!(dirs.first(), Some(&download), "{dirs:?}");
    }
}
