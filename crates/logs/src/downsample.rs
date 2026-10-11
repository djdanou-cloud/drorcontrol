//! Plot downsampling.
//!
//! * [`lttb`]: Largest-Triangle-Three-Buckets (Steinarsson 2013): keeps visual shape.
//! * [`minmax_lttb`]: MinMaxLTTB (Van Der Donckt et al., 2022): first keep each bucket's
//!   minimum and maximum, then run LTTB on that much smaller set. Narrow spikes (vibration,
//!   clipping, glitches) survive, and the cost is one O(n) pass plus LTTB on ~4×n_out points.

/// Indices of the points LTTB keeps (`n_out` ≥ 3; first and last always kept).
pub fn lttb(x: &[f64], y: &[f64], n_out: usize) -> Vec<usize> {
    let n = x.len().min(y.len());
    if n_out >= n || n_out < 3 {
        return (0..n).collect();
    }
    let mut out = Vec::with_capacity(n_out);
    out.push(0);
    let every = (n - 2) as f64 / (n_out - 2) as f64;
    let mut a = 0usize;
    for i in 0..n_out - 2 {
        let start = (i as f64 * every) as usize + 1;
        let end = (((i + 1) as f64 * every) as usize + 1).min(n - 1);
        // Average of the next bucket (or the last point).
        let (ns, ne) = (end, ((((i + 2) as f64) * every) as usize + 1).min(n));
        let (avg_x, avg_y) = if ns < ne {
            let k = (ne - ns) as f64;
            (
                x[ns..ne].iter().sum::<f64>() / k,
                y[ns..ne].iter().sum::<f64>() / k,
            )
        } else {
            (x[n - 1], y[n - 1])
        };
        let (ax, ay) = (x[a], y[a]);
        let mut best = start;
        let mut best_area = -1.0;
        for j in start..end.max(start + 1).min(n - 1) {
            let area = ((ax - avg_x) * (y[j] - ay) - (ax - x[j]) * (avg_y - ay)).abs();
            if area > best_area {
                best_area = area;
                best = j;
            }
        }
        out.push(best);
        a = best;
    }
    out.push(n - 1);
    out
}

/// Indices kept by MinMaxLTTB: min/max preselection over `ratio * n_out / 2` buckets, then
/// LTTB down to `n_out`. `ratio` 4 is the paper's recommendation.
pub fn minmax_lttb(x: &[f64], y: &[f64], n_out: usize, ratio: usize) -> Vec<usize> {
    let n = x.len().min(y.len());
    if n_out >= n || n_out < 3 {
        return (0..n).collect();
    }
    let buckets = (n_out * ratio / 2).max(1);
    if buckets * 2 >= n {
        return lttb(x, y, n_out);
    }
    let mut pre = Vec::with_capacity(buckets * 2 + 2);
    pre.push(0);
    let inner = n - 2;
    for b in 0..buckets {
        let s = 1 + b * inner / buckets;
        let e = 1 + (b + 1) * inner / buckets;
        if s >= e {
            continue;
        }
        let (mut lo, mut hi) = (s, s);
        for j in s..e {
            if y[j] < y[lo] {
                lo = j;
            }
            if y[j] > y[hi] {
                hi = j;
            }
        }
        pre.push(lo.min(hi));
        if lo != hi {
            pre.push(lo.max(hi));
        }
    }
    pre.push(n - 1);
    let px: Vec<f64> = pre.iter().map(|&i| x[i]).collect();
    let py: Vec<f64> = pre.iter().map(|&i| y[i]).collect();
    lttb(&px, &py, n_out).into_iter().map(|k| pre[k]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn series(n: usize) -> (Vec<f64>, Vec<f64>) {
        let x: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|v| (v * 0.01).sin()).collect();
        (x, y)
    }

    #[test]
    fn keeps_endpoints_and_count_and_order() {
        let (x, y) = series(100_000);
        for idx in [lttb(&x, &y, 1000), minmax_lttb(&x, &y, 1000, 4)] {
            assert_eq!(idx.len(), 1000);
            assert_eq!((idx[0], *idx.last().unwrap()), (0, 99_999));
            assert!(idx.windows(2).all(|w| w[0] < w[1]), "strictly increasing");
        }
    }

    #[test]
    fn short_input_is_returned_whole() {
        let (x, y) = series(50);
        assert_eq!(minmax_lttb(&x, &y, 1000, 4), (0..50).collect::<Vec<_>>());
    }

    #[test]
    fn minmax_keeps_a_single_sample_spike() {
        // A one-sample spike in 1 M points must survive downsampling to 500 points.
        let n = 1_000_000;
        let x: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let mut y = vec![0.0; n];
        y[654_321] = 100.0;
        assert!(minmax_lttb(&x, &y, 500, 4).contains(&654_321));
    }
}
