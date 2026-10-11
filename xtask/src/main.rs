//! DrorControl developer task runner.
//!
//! Run `cargo xtask help` for the command list. Every CI job calls these same commands,
//! so "green locally" and "green in CI" mean the same thing. Standard library only, so
//! the runner builds in seconds and adds no supply-chain surface.

use std::env;
use std::fmt;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

/// Tracked files at or above this size fail `repo-check` (coordination protocol §9).
const MAX_TRACKED_FILE_BYTES: u64 = 1024 * 1024;

const HELP: &str = "\
cargo xtask <command>

Commands:
  ci          Everything CI runs: fmt-check, lint, test, repo-check (stops at first failure)
  fmt         Format all Rust code in place
  fmt-check   Fail if any Rust code is not formatted
  lint        cargo clippy on all targets, warnings are errors
  test        cargo test for the whole workspace
  repo-check  Repository hygiene: generated skill files up to date, no oversized files
  doctor      Print toolchain and tool versions (for bug reports and onboarding)
  help        Show this message
";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Task {
    Ci,
    Fmt,
    FmtCheck,
    Lint,
    Test,
    RepoCheck,
    Doctor,
    Help,
}

impl Task {
    fn parse(arg: Option<&str>) -> Result<Self, String> {
        Ok(match arg {
            None | Some("help" | "-h" | "--help") => Task::Help,
            Some("ci") => Task::Ci,
            Some("fmt") => Task::Fmt,
            Some("fmt-check") => Task::FmtCheck,
            Some("lint") => Task::Lint,
            Some("test") => Task::Test,
            Some("repo-check") => Task::RepoCheck,
            Some("doctor") => Task::Doctor,
            Some(other) => return Err(format!("unknown command `{other}`")),
        })
    }
}

#[derive(Debug)]
struct Failure(String);

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

type TaskResult = Result<(), Failure>;
type Step = fn(&Path) -> TaskResult;

fn main() -> ExitCode {
    let arg = env::args().nth(1);
    let task = match Task::parse(arg.as_deref()) {
        Ok(task) => task,
        Err(err) => {
            eprintln!("error: {err}\n\n{HELP}");
            return ExitCode::from(2);
        }
    };
    let root = workspace_root();
    let result = match task {
        Task::Help => {
            print!("{HELP}");
            Ok(())
        }
        Task::Ci => ci(&root),
        Task::Fmt => cargo(&root, &["fmt", "--all"]),
        Task::FmtCheck => fmt_check(&root),
        Task::Lint => lint(&root),
        Task::Test => test(&root),
        Task::RepoCheck => repo_check(&root),
        Task::Doctor => {
            doctor(&root);
            Ok(())
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(failure) => {
            eprintln!("\nxtask: FAILED: {failure}");
            ExitCode::FAILURE
        }
    }
}

fn workspace_root() -> PathBuf {
    // xtask/ lives directly under the workspace root.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask has a parent directory")
        .to_path_buf()
}

fn ci(root: &Path) -> TaskResult {
    let steps: [(&str, Step); 4] = [
        ("fmt-check", fmt_check),
        ("lint", lint),
        ("test", test),
        ("repo-check", repo_check),
    ];
    for (name, step) in steps {
        println!("\n==> xtask {name}");
        step(root)?;
    }
    println!("\nxtask ci: all steps passed");
    Ok(())
}

fn fmt_check(root: &Path) -> TaskResult {
    cargo(root, &["fmt", "--all", "--check"])
}

fn lint(root: &Path) -> TaskResult {
    cargo(
        root,
        &[
            "clippy",
            "--workspace",
            "--all-targets",
            "--locked",
            "--",
            "-D",
            "warnings",
        ],
    )
}

fn test(root: &Path) -> TaskResult {
    cargo(root, &["test", "--workspace", "--locked"])
}

fn repo_check(root: &Path) -> TaskResult {
    let python = find_python().ok_or_else(|| {
        Failure("Python 3 not found (tried `python3`, `python`); needed for scripts/".into())
    })?;
    run(
        root,
        Command::new(&python).args(["scripts/sync_skill.py", "--check"]),
    )?;
    run(
        root,
        Command::new(&python).args(["scripts/gen_tokens.py", "--check"]),
    )?;
    run(
        root,
        Command::new(&python).args(["scripts/gen_adr_index.py", "--check"]),
    )?;

    let output = Command::new("git")
        .current_dir(root)
        .args(["ls-files", "-z"])
        .output()
        .map_err(|e| Failure(format!("cannot run git: {e}")))?;
    if !output.status.success() {
        return Err(Failure("`git ls-files` failed".into()));
    }
    let tracked = String::from_utf8_lossy(&output.stdout);
    let sizes = tracked
        .split('\0')
        .filter(|p| !p.is_empty())
        .filter_map(|p| Some((p, root.join(p).metadata().ok()?.len())));
    let oversized = oversized_files(sizes, MAX_TRACKED_FILE_BYTES);
    if !oversized.is_empty() {
        return Err(Failure(format!(
            "tracked files >= {} bytes (use the test-corpus mechanism instead): {}",
            MAX_TRACKED_FILE_BYTES,
            oversized.join(", ")
        )));
    }
    println!("repo-check: no tracked file >= {MAX_TRACKED_FILE_BYTES} bytes");
    Ok(())
}

fn oversized_files<'a>(files: impl Iterator<Item = (&'a str, u64)>, limit: u64) -> Vec<String> {
    files
        .filter(|&(_, size)| size >= limit)
        .map(|(path, size)| format!("{path} ({size} B)"))
        .collect()
}

fn doctor(root: &Path) {
    println!("workspace: {}", root.display());
    println!("host: {}-{}", std::env::consts::ARCH, std::env::consts::OS);
    let probes: [(&str, &[&str]); 8] = [
        ("rustc", &["--version"]),
        ("cargo", &["--version"]),
        ("rustup", &["--version"]),
        ("git", &["--version"]),
        ("python3", &["--version"]),
        ("python", &["--version"]),
        ("cmake", &["--version"]),
        ("ninja", &["--version"]),
    ];
    for (tool, args) in probes {
        let version = Command::new(tool)
            .args(args)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| first_line(&o.stdout, &o.stderr))
            .unwrap_or_else(|| "not found".into());
        println!("{tool:>8}: {version}");
    }
}

fn first_line(stdout: &[u8], stderr: &[u8]) -> String {
    // Some tools (older Python) print their version on stderr.
    let text = if stdout.is_empty() { stderr } else { stdout };
    String::from_utf8_lossy(text)
        .lines()
        .next()
        .unwrap_or("")
        .trim()
        .to_string()
}

fn find_python() -> Option<String> {
    ["python3", "python"]
        .into_iter()
        .find(|cmd| {
            Command::new(cmd).arg("--version").output().is_ok_and(|o| {
                o.status.success() && first_line(&o.stdout, &o.stderr).contains("Python 3")
            })
        })
        .map(str::to_string)
}

fn cargo(root: &Path, args: &[&str]) -> TaskResult {
    let cargo = env::var("CARGO").unwrap_or_else(|_| "cargo".into());
    run(root, Command::new(cargo).args(args))
}

fn run(root: &Path, cmd: &mut Command) -> TaskResult {
    cmd.current_dir(root);
    let shown = format!("{cmd:?}");
    let status = cmd
        .status()
        .map_err(|e| Failure(format!("cannot start {shown}: {e}")))?;
    if status.success() {
        Ok(())
    } else {
        Err(Failure(format!("{shown} exited with {status}")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_known_commands() {
        assert_eq!(Task::parse(Some("ci")), Ok(Task::Ci));
        assert_eq!(Task::parse(Some("repo-check")), Ok(Task::RepoCheck));
        assert_eq!(Task::parse(None), Ok(Task::Help));
        assert_eq!(Task::parse(Some("--help")), Ok(Task::Help));
    }

    #[test]
    fn rejects_unknown_command() {
        assert!(Task::parse(Some("deploy")).is_err());
    }

    #[test]
    fn flags_only_files_at_or_above_limit() {
        let files = [("small", 10), ("exact", 100), ("big", 101)];
        let found = oversized_files(files.into_iter(), 100);
        assert_eq!(found, vec!["exact (100 B)", "big (101 B)"]);
    }

    #[test]
    fn version_falls_back_to_stderr() {
        assert_eq!(first_line(b"", b"Python 3.8.0\n"), "Python 3.8.0");
        assert_eq!(first_line(b"cargo 1.97.0\nmore", b""), "cargo 1.97.0");
    }

    #[test]
    fn workspace_root_contains_manifest() {
        assert!(workspace_root().join("Cargo.toml").is_file());
    }
}
