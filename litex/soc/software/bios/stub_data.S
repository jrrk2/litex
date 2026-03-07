.section .rodata
.global _stub_bin_start, _stub_bin_end
.balign 4
_stub_bin_start:
.incbin "stub.bin"
_stub_bin_end:
