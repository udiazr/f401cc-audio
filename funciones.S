	.thumb

	.global do_sf4
do_sf4:
	
	push  {r3, r4, r5, r6, r7, lr}
	sub sp,#128
df401:
	str r0,[sp,#88]
	ldmia r0,{r1,r2,r3,r4,r5,r6}
	vldmia r1,{s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18,s19,s20,s21,s22,s23,s24,s25,s26,s27,s28,s29}
	str r1,[sp,#0] //xxx 
	str r6,[sp,#20] // &iii
	mov r0,r2  //audio
	mov r1,r3  //ileft
	mov r2,r4 //data
	mov r3,r5 //iii*2
df403:
	ldrh  r4,[r0]
	sxth  r4,r4
	add r0,#2
	vmov s3,r4
	vcvtr.f32.s32 s3,s3
	vmul.f32     s3,s3,s14
	ldrh  r5,[r0]
	sxth  r5,r5
	add   r0,#2
	vmov  s8,r5
	vcvtr.f32.s32 s8,s8
	vmul.f32      s8,s8,s18

/*	vadd.f32 s31,s3,s8
	vmul.f32 s31,s31,s20
	vmul.f32 s10,s10,s19
	vadd.f32 s10,s10,s31*/

   /*   xxx[6]=audio[i+1]*3; */
   /*   //   xxx[2]=audio[i]*5; */
   /*   //   xxx[6]=audio[i+1]*5; */
	/*   xxx[10]=(127*(int)xxx[10]+(int)xxx[2])>>7; */
	
	mov r7,#14
df402:
	vmul.f32  s31,s0,s17
	vmla.f32  s31,s1,s18
	vmla.f32  s31,s2,s19
	vmla.f32  s31,s3,s20
	vmla.f32  s31,s4,s21

	vmul.f32  s30,s0,s22
	vmla.f32  s30,s1,s23
	vmla.f32  s30,s2,s24
	vmla.f32  s30,s3,s25
	vmla.f32  s30,s4,s26

	vmul.f32  s15,s0,s27
	vmla.f32  s15,s1,s28
	vmla.f32  s15,s2,s29
	vsub.f32  s15,s15,s3
	vadd.f32  s15,s15,s4

	vmov      s0,s31
	vmov      s1,s30
	vmov      s2,s15

	vadd.f32  s31,s3,s0
	vcvtr.s32.f32 s31,s31
	vmov  r6,s31
	add   r6,#63
	strh  r6,[r2,r3]
	sub   r6,#63
	vmov  s31,r6
	vcvtr.f32.s32 s4,s31
	add   r3,#2





/*	vmul.f32  s31,s5,s17
	vmla.f32  s31,s6,s18
	vmla.f32  s31,s7,s19
	vmla.f32  s31,s8,s20
	vmla.f32  s31,s9,s21

	vmul.f32  s30,s5,s22
	vmla.f32  s30,s6,s23
	vmla.f32  s30,s7,s24
	vmla.f32  s30,s8,s25
	vmla.f32  s30,s9,s26

	vmul.f32  s15,s5,s27
	vmla.f32  s15,s6,s28
	vmla.f32  s15,s7,s29
	vsub.f32  s15,s15,s8
	vadd.f32  s15,s15,s9

	vmov      s5,s31
	vmov      s6,s30
	vmov      s7,s15

	vadd.f32  s31,s8,s5
	vcvtr.s32.f32 s31,s31
	vmov  r6,s31
	add   r6,#63*/
	mov   r6,#63
	strh  r6,[r2,r3]
	sub   r6,#63
	vmov  s31,r6
	vcvtr.f32.s32 s9,s31
	add   r3,#2

	
/*	vmul.f32  s31,s4,s12
	vmla.f32  s31,s5,s13
	vmla.f32  s31,s6,s14
	vmla.f32  s31,s7,s15
	vmul.f32  s30,s4,s16
	vmla.f32  s30,s5,s17
	vsub.f32  s30,s30,s6
	vadd.f32  s30,s7,s30
	vadd.f32  s29,s31,s6
	vmov.f32  s4,s31
	vmov.f32  s5,s30
	vcvtr.s32.f32 s29,s29
	vmov  r6,s29
	add   r6,#63
	mov   r6,#63
	strh  r6,[r2,r3]
	sub   r6,#63
//	vmov  s29,r6
//	vcvtr.f32.s32 s7,s29
	add   r3,#2*/

/*	vsub.f32 s31,s10,s11
	vadd.f32 s8,s8,s31
	vsub.f32 s31,s8,s11
	vadd.f32 s9,s9,s31
	vcvtr.s32.f32 s31,s9
	vmov  r6,s31
	add   r6,#63  */
	mov   r6,#63
	strh  r6,[r2,r3]
	sub   r6,#63
//	vmov  s29,r6
//	vcvtr.f32.s32 s11,s29
	add   r3,#2
	
	sub   r7,#1
	bne   df402
/*      xxx[0]+=xxx[2]-xxx[3]; */
/*      xxx[1]+=xxx[0]-xxx[3]; */
/*      tmp=(int)xxx[1]>>11; */
/*      data[iii]=tmp+period2; */
/*      xxx[3]=tmp<<11; */
df404:
	ldr r6,[sp,#20]
	asr r4,r3,#1
	str r4,[r6]
	sub r1,#2
	beq df499
	ldr r7,=df403
	mov pc,r7
df499:
	ldr r0,[sp,#0]
	vstmia r0,{s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13}
	add sp,#128
	pop    {r3, r4, r5, r6, r7, pc}




	.global do_sf3
do_sf3:
	
	push  {r3, r4, r5, r6, r7, lr}
	sub sp,#128
df301:
	str r0,[sp,#88]
	ldmia r0,{r1,r2,r3,r4,r5,r6}
	vldmia r1,{s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18,s19,s20}
	str r1,[sp,#0] //xxx 
	str r6,[sp,#20] // &iii
	mov r0,r2  //audio
	mov r1,r3  //ileft
	mov r2,r4 //data
	mov r3,r5 //iii*2
df303:
	ldrh  r4,[r0]
	sxth  r4,r4
	add r0,#2
	vmov s2,r4
	vcvtr.f32.s32 s2,s2
	vmul.f32     s2,s2,s18

	ldrh  r5,[r0]
	sxth  r5,r5
	add   r0,#2
	vmov  s6,r5
	vcvtr.f32.s32 s6,s6
	vmul.f32      s6,s6,s18

	vadd.f32 s31,s2,s6
	vmul.f32 s31,s31,s20
	vmul.f32 s10,s10,s19
	vadd.f32 s10,s10,s31

   /*   xxx[6]=audio[i+1]*3; */
   /*   //   xxx[2]=audio[i]*5; */
   /*   //   xxx[6]=audio[i+1]*5; */
	/*   xxx[10]=(127*(int)xxx[10]+(int)xxx[2])>>7; */
	
	mov r7,#14
df302:
	vmul.f32  s31,s0,s12
	vmla.f32  s31,s1,s13
	vmla.f32  s31,s2,s14
	vmla.f32  s31,s3,s15
	vmul.f32  s30,s0,s16
	vmla.f32  s30,s1,s17
	vsub.f32  s30,s30,s2
	vadd.f32  s30,s3,s30
	vadd.f32  s29,s31,s2
	vmov.f32  s0,s31
	vmov.f32  s1,s30
	vcvtr.s32.f32 s29,s29
	vmov  r6,s29
	add   r6,#63
	strh  r6,[r2,r3]
	sub   r6,#63
	vmov  s29,r6
	vcvtr.f32.s32 s3,s29
	add   r3,#2

	vmul.f32  s31,s4,s12
	vmla.f32  s31,s5,s13
	vmla.f32  s31,s6,s14
	vmla.f32  s31,s7,s15
	vmul.f32  s30,s4,s16
	vmla.f32  s30,s5,s17
	vsub.f32  s30,s30,s6
	vadd.f32  s30,s7,s30
	vadd.f32  s29,s31,s6
	vmov.f32  s4,s31
	vmov.f32  s5,s30
	vcvtr.s32.f32 s29,s29
	vmov  r6,s29
	add   r6,#63
	strh  r6,[r2,r3]
	sub   r6,#63
	vmov  s29,r6
	vcvtr.f32.s32 s7,s29
	add   r3,#2

	vsub.f32 s31,s10,s11
	vadd.f32 s8,s8,s31
	vsub.f32 s31,s8,s11
	vadd.f32 s9,s9,s31
	vcvtr.s32.f32 s31,s9
	vmov  r6,s31
	add   r6,#63
	strh  r6,[r2,r3]
	sub   r6,#63
	vmov  s29,r6
	vcvtr.f32.s32 s11,s29
	add   r3,#2
	
	sub   r7,#1
	bne   df302
/*      xxx[0]+=xxx[2]-xxx[3]; */
/*      xxx[1]+=xxx[0]-xxx[3]; */
/*      tmp=(int)xxx[1]>>11; */
/*      data[iii]=tmp+period2; */
/*      xxx[3]=tmp<<11; */
df304:
	ldr r6,[sp,#20]
	asr r4,r3,#1
	str r4,[r6]
	sub r1,#2
	beq df399
	ldr r7,=df303
	mov pc,r7
df399:
	ldr r0,[sp,#0]
	vstmia r0,{s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11}
	add sp,#128
	pop    {r3, r4, r5, r6, r7, pc}


