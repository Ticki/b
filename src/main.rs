#![feature(core)]
#![feature(core_intrinsics)]

extern crate core;
mod memory;

fn main() {
    unsafe {
    memory::memory_init();
    println!("1. Alloc 400 bytes");
    let a = memory::alloc(400);

    println!("2. Alloc 40 bytes");
    let b = memory::alloc(40);

    assert!(a != b);

    println!("Dealloc 400");
    let d = memory::unalloc(a);
    }

    println!("Hello, world!");
}
