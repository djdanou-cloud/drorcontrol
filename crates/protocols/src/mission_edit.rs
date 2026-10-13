//! Editing a mission in the planner (plan 04, mission planning): insert, delete, move and
//! change items with undo and redo. Home (item 0) stays first. `DO_JUMP` (177) targets are
//! item numbers, so they are renumbered with every insert, delete and move to keep pointing
//! at the same item; a jump whose target is deleted then points at the item that took its
//! place, and the edit says so. Nothing here talks to a vehicle: the result is uploaded
//! through the usual verified path.

use crate::mission::MissionItem;

const DO_JUMP: u16 = 177;
/// Undo steps kept.
pub const UNDO_DEPTH: usize = 200;

/// Why an edit was not made.
#[derive(Debug, Clone, PartialEq)]
pub enum EditError {
    /// Item 0 is home and stays first.
    Home,
    /// No such item.
    OutOfRange(usize),
}

impl std::fmt::Display for EditError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EditError::Home => f.write_str("item 0 is home; it cannot be moved or deleted"),
            EditError::OutOfRange(i) => write!(f, "there is no item {i}"),
        }
    }
}

/// A mission being edited.
#[derive(Debug, Clone, Default)]
pub struct MissionEditor {
    items: Vec<MissionItem>,
    undo: Vec<(String, Vec<MissionItem>)>,
    redo: Vec<(String, Vec<MissionItem>)>,
}

impl MissionEditor {
    /// Start editing `items` (item 0 is home).
    pub fn new(items: Vec<MissionItem>) -> Self {
        MissionEditor {
            items,
            ..Default::default()
        }
    }

    /// The mission as it now is.
    pub fn items(&self) -> &[MissionItem] {
        &self.items
    }

    /// What `undo` would undo.
    pub fn undo_label(&self) -> Option<&str> {
        self.undo.last().map(|u| u.0.as_str())
    }

    /// What `redo` would redo.
    pub fn redo_label(&self) -> Option<&str> {
        self.redo.last().map(|r| r.0.as_str())
    }

    fn record(&mut self, label: String) {
        self.undo.push((label, self.items.clone()));
        if self.undo.len() > UNDO_DEPTH {
            self.undo.remove(0);
        }
        self.redo.clear();
    }

    /// Undo the last edit; its label.
    pub fn undo(&mut self) -> Option<String> {
        let (label, before) = self.undo.pop()?;
        let now = std::mem::replace(&mut self.items, before);
        self.redo.push((label.clone(), now));
        Some(label)
    }

    /// Redo the last undone edit; its label.
    pub fn redo(&mut self) -> Option<String> {
        let (label, after) = self.redo.pop()?;
        let now = std::mem::replace(&mut self.items, after);
        self.undo.push((label.clone(), now));
        Some(label)
    }

    fn editable(&self, at: usize) -> Result<(), EditError> {
        if at == 0 {
            Err(EditError::Home)
        } else if at >= self.items.len() {
            Err(EditError::OutOfRange(at))
        } else {
            Ok(())
        }
    }

    /// Renumber jump targets with `map` (old item number → new); the jumps whose target
    /// had no new number are returned (by their new item number) and left pointing at
    /// `fallback(old)`.
    fn remap_jumps(
        &mut self,
        map: impl Fn(usize) -> Option<usize>,
        fallback: impl Fn(usize) -> usize,
    ) -> Vec<usize> {
        let mut lost = Vec::new();
        for (k, i) in self.items.iter_mut().enumerate() {
            if i.command != DO_JUMP || i.params[0] < 1.0 || i.params[0].fract() != 0.0 {
                continue;
            }
            let old = i.params[0] as usize;
            let new = map(old).unwrap_or_else(|| {
                lost.push(k);
                fallback(old)
            });
            i.params[0] = new as f32;
        }
        lost
    }

    /// Insert `item` at `at` (1..=len); later items move down one.
    pub fn insert(&mut self, at: usize, item: MissionItem) -> Result<(), EditError> {
        if at == 0 {
            return Err(EditError::Home);
        }
        if at > self.items.len() {
            return Err(EditError::OutOfRange(at));
        }
        self.record(format!("insert item {at}"));
        self.items.insert(at, item);
        self.remap_jumps(|t| Some(if t >= at { t + 1 } else { t }), |t| t);
        // The new item itself, if a jump, keeps the target it was given.
        Ok(())
    }

    /// Delete item `at`. Returns the jumps (by item number) whose target this was: they now
    /// point at the item that took its place.
    pub fn delete(&mut self, at: usize) -> Result<Vec<usize>, EditError> {
        self.editable(at)?;
        self.record(format!("delete item {at}"));
        self.items.remove(at);
        Ok(self.remap_jumps(
            |t| match t.cmp(&at) {
                std::cmp::Ordering::Less => Some(t),
                std::cmp::Ordering::Equal => None,
                std::cmp::Ordering::Greater => Some(t - 1),
            },
            |t| t,
        ))
    }

    /// Move item `from` to position `to` (both 1..len).
    pub fn move_item(&mut self, from: usize, to: usize) -> Result<(), EditError> {
        self.editable(from)?;
        self.editable(to)?;
        if from == to {
            return Ok(());
        }
        self.record(format!("move item {from} to {to}"));
        let it = self.items.remove(from);
        self.items.insert(to, it);
        let map = |t: usize| {
            Some(if t == from {
                to
            } else if from < to && t > from && t <= to {
                t - 1
            } else if to < from && t >= to && t < from {
                t + 1
            } else {
                t
            })
        };
        self.remap_jumps(map, |t| t);
        Ok(())
    }

    /// Replace item `at` (e.g. a changed position, altitude or command).
    pub fn replace(&mut self, at: usize, item: MissionItem) -> Result<(), EditError> {
        self.editable(at)?;
        if self.items[at] == item {
            return Ok(());
        }
        self.record(format!("change item {at}"));
        self.items[at] = item;
        Ok(())
    }

    /// Set every waypoint-like item above home (frame 3) to `alt_m`: one undo step.
    /// Returns how many changed.
    pub fn set_all_altitudes(&mut self, alt_m: f32) -> usize {
        let targets: Vec<usize> = (1..self.items.len())
            .filter(|&k| {
                let i = &self.items[k];
                i.frame == 3 && crate::mission::has_location(i.command) && i.z != alt_m
            })
            .collect();
        if targets.is_empty() {
            return 0;
        }
        self.record(format!("set {} altitudes to {alt_m} m", targets.len()));
        for &k in &targets {
            self.items[k].z = alt_m;
        }
        targets.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wp(n: i32) -> MissionItem {
        MissionItem {
            frame: 3,
            command: 16,
            params: [0.0; 4],
            x: -353_600_000 + n,
            y: 1_491_600_000,
            z: 30.0,
            autocontinue: true,
        }
    }

    fn jump(to: f32) -> MissionItem {
        MissionItem {
            frame: 2,
            command: DO_JUMP,
            params: [to, 2.0, 0.0, 0.0],
            x: 0,
            y: 0,
            z: 0.0,
            autocontinue: true,
        }
    }

    /// home, wp1, wp2, wp3, jump→2
    fn sample() -> MissionEditor {
        MissionEditor::new(vec![wp(0), wp(1), wp(2), wp(3), jump(2.0)])
    }

    fn target(e: &MissionEditor) -> f32 {
        e.items()
            .iter()
            .find(|i| i.command == DO_JUMP)
            .unwrap()
            .params[0]
    }

    #[test]
    fn jumps_follow_their_target() {
        let mut e = sample();
        e.insert(1, wp(9)).unwrap();
        assert_eq!(target(&e), 3.0, "wp2 is now item 3");
        assert_eq!(e.items()[3], wp(2));
        e.move_item(3, 1).unwrap();
        assert_eq!(e.items()[1], wp(2));
        assert_eq!(target(&e), 1.0);
        e.move_item(1, 4).unwrap();
        assert_eq!(e.items()[4], wp(2));
        assert_eq!(target(&e), 4.0);
        // Deleting an item before the target shifts it up.
        e.delete(1).unwrap();
        assert_eq!(e.items()[3], wp(2));
        assert_eq!(target(&e), 3.0);
        // Deleting the target itself is reported; the jump points at the next item.
        let j = e.items().iter().position(|i| i.command == DO_JUMP).unwrap();
        let lost = e.delete(3).unwrap();
        assert_eq!(lost, [j - 1]);
        assert_eq!(target(&e), 3.0);
    }

    #[test]
    fn undo_and_redo_restore_exactly() {
        let mut e = sample();
        let start = e.items().to_vec();
        e.insert(2, wp(7)).unwrap();
        e.delete(1).unwrap();
        assert_eq!(e.set_all_altitudes(50.0), 3);
        let end = e.items().to_vec();
        assert_eq!(e.undo_label(), Some("set 3 altitudes to 50 m"));
        assert_eq!(e.undo().as_deref(), Some("set 3 altitudes to 50 m"));
        assert_eq!(e.undo().as_deref(), Some("delete item 1"));
        assert_eq!(e.undo().as_deref(), Some("insert item 2"));
        assert_eq!(e.items(), start);
        assert_eq!(e.undo(), None);
        e.redo();
        e.redo();
        e.redo();
        assert_eq!(e.items(), end);
        // A new edit after undo drops the redo history.
        e.undo();
        e.replace(1, wp(5)).unwrap();
        assert_eq!(e.redo(), None);
        // Home is fixed; out of range is refused; nothing recorded.
        let n = e.undo.len();
        assert_eq!(e.delete(0), Err(EditError::Home));
        assert_eq!(e.move_item(1, 0), Err(EditError::Home));
        assert_eq!(e.insert(0, wp(1)), Err(EditError::Home));
        assert_eq!(e.delete(99), Err(EditError::OutOfRange(99)));
        assert_eq!(e.undo.len(), n);
    }

    #[test]
    fn undo_history_is_bounded() {
        let mut e = sample();
        for k in 0..(UNDO_DEPTH + 10) {
            e.replace(1, wp(100 + k as i32)).unwrap();
        }
        assert_eq!(e.undo.len(), UNDO_DEPTH);
    }
}
