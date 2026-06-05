use esp_hal::time::{Duration, Instant};

pub fn delay_ms(ms: u64) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(ms) {}
}
