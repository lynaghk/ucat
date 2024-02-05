// lol rust https://stackoverflow.com/questions/53619695/calculating-maximum-value-of-a-set-of-constant-expressions-at-compile-time
pub const fn max(a: usize, b: usize) -> usize {
    [a, b][(a < b) as usize]
}
