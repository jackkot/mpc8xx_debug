#include "ppc_asm.h"

	.text
	.global _start
_start:
	b _start1

p_ep:
	/* 0 */ .long	ep_simple
	/* 1 */ .long	ep_memcpy
	/* 2 */ .long	ep_flash_write_one
	/* 3 */ .long	ep_flash_read_one
	/* 4 */ .long	ep_flash_erase
	/* 5 */ .long	ep_flash_program

p_start:	.long	_start
p_etext:	.long	_etext
p_bss_start:	.long	__bss_start
p_end:		.long	_end
p_pstack:	.long	0xfff03ffc

_start1:
	bl .+4
p_base:	mflr	r10		/* r10 now points to runtime addr of p_base */
	/* Do a cache flush for our text, in case the loader didn't */
	lwz	r9,p_start-p_base(r10)	/* note: these are relocated now */
	lwz	r8,p_etext-p_base(r10)
1:	dcbf	r0,r9
	icbi	r0,r9
	addi	r9,r9,0x20
	cmplw	cr0,r9,r8
	blt	1b
	sync
	isync

	/* Clear the BSS */
	lwz	r9,p_bss_start-p_base(r10)
	lwz	r8,p_end-p_base(r10)
	li	r0,0
2:	stw	r0,0(r9)
	addi	r9,r9,4
	cmplw	cr0,r9,r8
	blt	2b

	/* Possibly set up a custom stack */
	lwz	r1,p_pstack-p_base(r10)
	li	r0,0
	stwu	r0,-16(r1)	/* establish a stack frame */

	/* Select entry point from p_ep array */
	rlwinm  r31, r31, 2, 0, 29 /* r31 <<= 2 */
	addi	r31, r31, p_ep-p_base
	lwzx	r31, r31, r10
	/* Jump to entry point */
	mtctr	r31
	bctrl
	
	.global __exit
__exit:
	trap
	b .+0
