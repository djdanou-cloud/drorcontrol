//! Append-only, hash-chained audit log of commands (plan 08 §4.2).
//!
//! One record per line, tab-separated:
//! `unix_us  sysid  command_id  text  prev_hash  hash`, where
//! `hash = SHA-256(prev_hash ‖ "\t" ‖ unix_us ‖ "\t" ‖ sysid ‖ "\t" ‖ command_id ‖ "\t" ‖ text)`
//! in lowercase hex and the first record's `prev_hash` is 64 zeros. Editing, removing or
//! reordering any record breaks every later hash; [`verify`] finds the first broken line.

use std::io::{self, BufRead, Write};

use sha2::{Digest, Sha256};

use crate::command::{AuditEvent, CommandId};

/// `prev_hash` of the first record.
pub const GENESIS: &str = "0000000000000000000000000000000000000000000000000000000000000000";

fn hash(prev: &str, body: &str) -> String {
    let mut h = Sha256::new();
    h.update(prev.as_bytes());
    h.update(b"\t");
    h.update(body.as_bytes());
    h.finalize().iter().map(|b| format!("{b:02x}")).collect()
}

fn clean(s: &str) -> String {
    s.replace(['\t', '\n', '\r'], " ")
}

/// Writes records, continuing the chain from `last_hash`.
pub struct AuditLog<W: Write> {
    out: W,
    last: String,
    records: u64,
}

impl<W: Write> AuditLog<W> {
    /// Start a new chain.
    pub fn new(out: W) -> Self {
        Self::resume(out, GENESIS.to_string())
    }

    /// Continue a chain whose last record hash is `last_hash` (see [`last_hash`]).
    pub fn resume(out: W, last_hash: String) -> Self {
        AuditLog {
            out,
            last: last_hash,
            records: 0,
        }
    }

    /// Append one event (wall time `unix_us`) and flush.
    pub fn append(&mut self, unix_us: u64, e: &AuditEvent) -> io::Result<()> {
        let id = e.id.map_or("-".to_string(), |CommandId(n)| n.to_string());
        let body = format!("{unix_us}\t{}\t{id}\t{}", e.sysid, clean(&e.text));
        let h = hash(&self.last, &body);
        writeln!(self.out, "{body}\t{}\t{h}", self.last)?;
        self.out.flush()?;
        self.last = h;
        self.records += 1;
        Ok(())
    }

    /// Records written by this writer.
    pub fn records(&self) -> u64 {
        self.records
    }

    /// The writer.
    pub fn into_inner(self) -> W {
        self.out
    }
}

/// Where a chain is broken.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Broken {
    /// 1-based line number.
    pub line: usize,
    /// What is wrong.
    pub reason: String,
}

/// Check a whole log. Returns the number of records and the last hash.
pub fn verify(input: impl BufRead) -> Result<(usize, String), Broken> {
    let mut prev = GENESIS.to_string();
    let mut n = 0;
    for (i, line) in input.lines().enumerate() {
        let bad = |reason: &str| Broken {
            line: i + 1,
            reason: reason.to_string(),
        };
        let line = line.map_err(|e| bad(&e.to_string()))?;
        let cols: Vec<&str> = line.split('\t').collect();
        if cols.len() != 6 {
            return Err(bad("expected 6 tab-separated fields"));
        }
        if cols[4] != prev {
            return Err(bad("prev_hash does not match the previous record"));
        }
        if hash(&prev, &cols[..4].join("\t")) != cols[5] {
            return Err(bad("hash does not match the record"));
        }
        prev = cols[5].to_string();
        n += 1;
    }
    Ok((n, prev))
}

/// Last hash of an existing, valid log (to [`AuditLog::resume`] it).
pub fn last_hash(input: impl BufRead) -> Result<String, Broken> {
    verify(input).map(|(_, h)| h)
}
