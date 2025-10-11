use std::ffi::NulError;
use std::fmt::{Display, Formatter};

/// Represents failures that can occur while interacting with the Bullet physics server.
#[derive(Debug)]
pub enum BulletError {
    /// Attempted to connect but Bullet returned a null pointer.
    NullPointer(&'static str),
    /// Bullet reported that the physics server is not responsive.
    ServerUnavailable(&'static str),
    /// The command completed with an unexpected status code.
    UnexpectedStatus { expected: i32, actual: i32 },
    /// A low-level FFI command returned an error code.
    CommandFailed { message: &'static str, code: i32 },
    /// Converting Rust strings into C strings failed.
    CString(NulError),
}

impl Display for BulletError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            BulletError::NullPointer(msg) => write!(f, "{msg}"),
            BulletError::ServerUnavailable(msg) => write!(f, "{msg}"),
            BulletError::UnexpectedStatus { expected, actual } => write!(
                f,
                "Unexpected Bullet status. expected={expected} actual={actual}"
            ),
            BulletError::CommandFailed { message, code } => {
                write!(f, "{message} (code={code})")
            }
            BulletError::CString(err) => err.fmt(f),
        }
    }
}

impl std::error::Error for BulletError {}

impl From<NulError> for BulletError {
    fn from(value: NulError) -> Self {
        BulletError::CString(value)
    }
}

pub type BulletResult<T> = Result<T, BulletError>;
