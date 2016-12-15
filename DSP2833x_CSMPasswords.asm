	
;----------------------------------------------------------------------
; Application lock keys (locations just used in bootloading for unlocking)

						.global _App_Key0
						.global _App_Key1
						.global _App_Key2
						.global _App_Key3
						.global _App_Key4
						.global _App_Key5
						.global _App_Key6
						.global _App_Key7

	  .sect "keystore"
;pass is:  "CyberWandLockKey" in ASCII
_App_Key0      .int	0x4379		;PWL0 (LSW of 128-bit password)
_App_Key1      .int	0x6265		;PWL1
_App_Key2      .int	0x7257		;PWL2
_App_Key3      .int	0x616E		;PWL3
_App_Key4      .int	0x644C		;PWL4
_App_Key5      .int	0x6F63		;PWL5
_App_Key6      .int	0x6B4B		;PWL6
_App_Key7      .int	0x6579		;PWL7 (MSW of 128-bit password)						

;----------------------------------------------------------------------

; To unlock the code security module, a dummy read of the flash password
; locations is required. The following code performs this read.
;
; To call this function in C, use: CsmPwlDummyRead()
;
; The C prototype of this function is: void CsmPwlDummyRead(void)

      .text
      .def _CsmPwlDummyRead
      .ref _CsmPwl
_CsmPwlDummyRead:
      MOVW    DP,#_CsmPwl
	  MOV     AL,@_CsmPwl
	  MOV     AL,@_CsmPwl+1
	  MOV     AL,@_CsmPwl+2
	  MOV     AL,@_CsmPwl+3
	  MOV     AL,@_CsmPwl+4
	  MOV     AL,@_CsmPwl+5
	  MOV     AL,@_CsmPwl+6
	  MOV     AL,@_CsmPwl+7
      LRETR


;//===========================================================================
;// End of file.
;//===========================================================================

      
