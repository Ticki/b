#![feature(core)]
#![feature(core_intrinsics)]

extern crate core;
mod memory;

fn main() {
    unsafe {
        memory::memory_init();


        for i in 512..1024 {
            println!("n: {}", i);
            let b = memory::alloc(i);


        }
    }

    println!("Hello, world!");
}
