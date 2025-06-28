use core::{panic::PanicInfo, sync::atomic::{compiler_fence, Ordering}};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    // Reset the CPU!
    loop {
        compiler_fence(Ordering::SeqCst);
    }
}