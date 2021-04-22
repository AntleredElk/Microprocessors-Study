.syntax unified


.global kalmanASS

kalmanASS:
	/*
	 * Returns 0 in r0 if successful
	 * Returns -1 in r0 if unsuccessful (overflow,underflow, division by zero...)
	 */


	//r0 -> pointer to the struct, also is the pointer to q since that is the first float in the struct
	//s0 -> measurement float
	//s1 -> q
	//s2 -> r
	//s3 -> x
	//s4 -> p
	//s5 -> k
	//s6 -> used for intermediate in math

	//CALLEE-SAVE CONVENTION
	vpush {s1-s6}

	vldr s1, [r0] //q
	vldr s2, [r0, #4] //r
	vldr s3, [r0, #8] //x
	vldr s4, [r0, #12] //p
	vldr s5, [r0, #16] //k

	//p = p + q ---> s4 = s4 + s1
	vadd.f32 s4, s4, s1

	//k = p/(p+r)
	vadd.f32 s6, s4, s2
	vdiv.f32 s5, s4, s6
	vstr s5, [r0, #16] //Store new k

	//x = x + k(measure - x)
	vsub.f32 s6, s0, s3
	vmul.f32 s6, s5, s6
	vadd.f32 s3, s3, s6
	vstr s3, [r0, #8] //Store new x

	//p = (1-k)p
	vmov s6, #1.0
	vsub.f32 s6, s6, s5
	vmul.f32 s4, s6, s4
	vstr s4, [r0, #12] //Store new p

	//Check if errors occurred
	vmrs r1, fpscr
	and r1, r1, 0x0000000F
	cmp r1, #0
	bgt error

	//CALLEE-SAVE CONVENTION
	vpop {s1-s6}
	mov r0, #0
	bx lr

error:
	//Return -1
	mov r0, #-1
	vpop {s1-s6}
	bx lr
