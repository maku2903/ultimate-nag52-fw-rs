use embassy_time::Instant;

defmt::timestamp!("{=u64:tms}", {
    Instant::now().as_millis()
});