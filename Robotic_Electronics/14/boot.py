# boot.py — run once at boot, before main.py
import micropython

# Allow clear error messages if something goes wrong inside IRQs
micropython.alloc_emergency_exception_buf(100)

print("boot.py: system ready")
 