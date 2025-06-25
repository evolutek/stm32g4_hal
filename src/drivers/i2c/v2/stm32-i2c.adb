------------------------------------------------------------------------------
--                                                                          --
--                    Copyright (C) 2015, AdaCore                           --
--                                                                          --
--  Redistribution and use in source and binary forms, with or without      --
--  modification, are permitted provided that the following conditions are  --
--  met:                                                                    --
--     1. Redistributions of source code must retain the above copyright    --
--        notice, this list of conditions and the following disclaimer.     --
--     2. Redistributions in binary form must reproduce the above copyright --
--        notice, this list of conditions and the following disclaimer in   --
--        the documentation and/or other materials provided with the        --
--        distribution.                                                     --
--     3. Neither the name of STMicroelectronics nor the names of its       --
--        contributors may be used to endorse or promote products derived   --
--        from this software without specific prior written permission.     --
--                                                                          --
--   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    --
--   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      --
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  --
--   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   --
--   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, --
--   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       --
--   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  --
--   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  --
--   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    --
--   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  --
--   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   --
--                                                                          --
--                                                                          --
--  This file is based on:                                                  --
--                                                                          --
--   @file    stm32f4xx_hal_i2c.c                                           --
--   @author  MCD Application Team                                          --
--   @version V1.1.0                                                        --
--   @date    19-June-2014                                                  --
--   @brief   I2C HAL module driver.                                        --
--                                                                          --
--   COPYRIGHT(c) 2014 STMicroelectronics                                   --
------------------------------------------------------------------------------

with Ada.Real_Time; use Ada.Real_Time;

with STM32_SVD.I2C; use STM32_SVD.I2C;
with STM32.Device;  use STM32.Device;

package body STM32.I2C is

   use type HAL.I2C.I2C_Status;

   subtype Dispatch is I2C_Port'Class;

   ---------------
   -- Configure --
   ---------------

   procedure Configure (This : in out I2C_Port; Conf : I2C_Configuration) is
      CR1        : CR1_Register;
      --       CCR        : CCR_Register;
      OAR1       : OAR1_Register;
      PCLK1      : constant UInt32 := System_Clock_Frequencies.PCLK1;
      Freq_Range : constant UInt16 := UInt16 (PCLK1 / 1_000_000);

   begin
      if This.State /= Reset then
         return;
      end if;

      This.Config := Conf;

      --  Disable the I2C port
      if Freq_Range < 8 or else Freq_Range > 48 then
         raise Program_Error
           with
             "PCLK1 too high or too low: expected 8-48 MHz, current"
             & Freq_Range'Img
             & " MHz";
      end if;

      Set_State (This, False);

      --  Set the port timing
      case Conf.Clock_Speed is
         when 10_000 =>
            This.Periph.TIMINGR.SCLL := 16#C7#;
            This.Periph.TIMINGR.SCLH := 16#C3#;
            This.Periph.TIMINGR.SDADEL := 16#2#;
            This.Periph.TIMINGR.SCLDEL := 16#4#;
            case Freq_Range is
               when 8 =>
                  This.Periph.TIMINGR.PRESC := 16#1#;

               when 16 =>
                  This.Periph.TIMINGR.PRESC := 16#3#;

               when 48 =>
                  This.Periph.TIMINGR.PRESC := 16#B#;

               when others =>
                  raise Program_Error
                    with "Unsupported APB Clock" & Freq_Range'Img;
            end case;

         when 100_000 =>
            --  Mode selection to Standard Mode
            This.Periph.TIMINGR.SCLL := 16#13#;
            This.Periph.TIMINGR.SCLH := 16#F#;
            This.Periph.TIMINGR.SDADEL := 16#2#;
            This.Periph.TIMINGR.SCLDEL := 16#4#;
            case Freq_Range is
               when 8 =>
                  This.Periph.TIMINGR.PRESC := 16#1#;

               when 16 =>
                  This.Periph.TIMINGR.PRESC := 16#3#;

               when 48 =>
                  This.Periph.TIMINGR.PRESC := 16#B#;

               when others =>
                  raise Program_Error
                    with "Unsupported APB Clock" & Freq_Range'Img;
            end case;

         when 400_000 =>
            --  Mode selection to Fast Mode
            This.Periph.TIMINGR.SCLL := 16#9#;
            This.Periph.TIMINGR.SCLH := 16#3#;
            This.Periph.TIMINGR.SCLDEL := 16#3#;
            case Freq_Range is
               when 8 =>
                  This.Periph.TIMINGR.PRESC := 16#0#;
                  This.Periph.TIMINGR.SDADEL := 16#1#;

               when 16 =>
                  This.Periph.TIMINGR.PRESC := 16#1#;
                  This.Periph.TIMINGR.SDADEL := 16#2#;

               when 48 =>
                  This.Periph.TIMINGR.PRESC := 16#5#;
                  This.Periph.TIMINGR.SDADEL := 16#3#;

               when others =>
                  raise Program_Error
                    with "Unsupported APB Clock" & Freq_Range'Img;
            end case;
            --          when 1_000_000 =>
            --  Mode selection to Fast Mode Plus

         when others =>
            raise Program_Error
              with "Unsupported I2C Clock Speed" & Conf.Clock_Speed'Img;
      end case;

      --  CR1 configuration
      case Conf.Mode is
         when I2C_Mode =>
            CR1.ALERTEN := False;
            CR1.SMBDEN := False;
            CR1.SMBHEN := False;

         when SMBusDevice_Mode =>
            raise Program_Error with "Unsupported I2C Mode" & Conf.Mode'Img;
            --             CR1.SMBUS := True;
            --             CR1.SMBTYPE := False;

         when SMBusHost_Mode =>
            --             CR1.SMBUS := True;
            --             CR1.SMBTYPE := True;
            raise Program_Error with "Unsupported I2C Mode" & Conf.Mode'Img;
      end case;

      CR1.GCEN := Conf.General_Call_Enabled;
      CR1.NOSTRETCH := not Conf.Clock_Stretching_Enabled;
      This.Periph.CR1 := CR1;

      --  Address mode (slave mode) configuration
      OAR1.OA1MODE := Conf.Addressing_Mode = Addressing_Mode_10bit;
      OAR1.OA1 := Conf.Own_Address;
      --       case Conf.Addressing_Mode is
      --          when Addressing_Mode_7bit =>
      --             OAR1.ADD7  := UInt7 (Conf.Own_Address / 2);
      --             OAR1.ADD10 := 0;
      --          when Addressing_Mode_10bit =>
      --             OAR1.ADD0  := (Conf.Own_Address and 2#1#) /= 0;
      --             OAR1.ADD7  := UInt7 ((Conf.Own_Address / 2) and 2#1111111#);
      --             OAR1.ADD10 := UInt2 (Conf.Own_Address / 2 ** 8);
      --       end case;

      This.Periph.OAR1.OA1EN := False;
      This.Periph.OAR2.OA2EN := False;
      This.Periph.OAR1 := OAR1;
      This.Periph.OAR1.OA1EN := True;

      --  This.DMA_Enabled := Conf.Enable_DMA;

      Set_State (This, True);
      This.State := Ready;
   end Configure;

   -----------------
   -- Flag_Status --
   -----------------

   function Flag_Status
     (This : I2C_Port; Flag : I2C_Status_Flag) return Boolean is
   begin
      case Flag is
         when Tx_Data_Register_Empty =>
            return This.Periph.ISR.TXE;

         when Tx_Interrupt_Status =>
            return This.Periph.ISR.TXIS;

         when Rx_Data_Register_Not_Empty =>
            return This.Periph.ISR.RXNE;

         when Address_Match =>
            return This.Periph.ISR.ADDR;

         when Ack_Failure =>
            return This.Periph.ISR.NACKF;

         when Stop_Detection =>
            return This.Periph.ISR.STOPF;

         when Tx_Complete =>
            return This.Periph.ISR.TC;

         when Tx_Complete_Reload =>
            return This.Periph.ISR.TCR;

         when Bus_Error =>
            return This.Periph.ISR.BERR;

         when Arbitration_Lost =>
            return This.Periph.ISR.ARLO;

         when UnderOverrun =>
            return This.Periph.ISR.OVR;

         when Packet_Error =>
            return This.Periph.ISR.PECERR;

         when Timeout =>
            return This.Periph.ISR.TIMEOUT;

         when SMB_Alert =>
            return This.Periph.ISR.ALERT;

         when Busy =>
            return This.Periph.ISR.BUSY;

         when Tx_Direction =>
            return This.Periph.ISR.DIR;
      end case;
   end Flag_Status;

   ----------------
   -- Clear_Flag --
   ----------------

   procedure Clear_Flag
     (Port : in out I2C_Port; Target : Clearable_I2C_Status_Flag)
   is
      -- Unref  : Bit with Unreferenced;
   begin
      case Target is
         when Address_Match =>
            Port.Periph.ICR.ADDRCF := True;

         when Ack_Failure =>
            Port.Periph.ICR.NACKCF := True;

         when Stop_Detection =>
            Port.Periph.ICR.STOPCF := True;

         when Bus_Error =>
            Port.Periph.ICR.BERRCF := True;

         when Arbitration_Lost =>
            Port.Periph.ICR.ARLOCF := True;

         when UnderOverrun =>
            Port.Periph.ICR.OVRCF := True;

         when Packet_Error =>
            Port.Periph.ICR.PECCF := True;

         when Timeout =>
            Port.Periph.ICR.TIMOUTCF := True;

         when SMB_Alert =>
            Port.Periph.ICR.ALERTCF := True;
      end case;
   end Clear_Flag;

   procedure Wait_While_Flag
     (This    : in out I2C_Port;
      Flag    : I2C_Status_Flag;
      F_State : Boolean;
      Timeout : Natural;
      Status  : out HAL.I2C.I2C_Status)
   is
      Deadline : constant Time := Clock + Milliseconds (Timeout);
   begin
      while Flag_Status (This, Flag) = F_State loop
         if Clock > Deadline then
            This.State := Ready;
            Status := HAL.I2C.Err_Timeout;
            return;
         end if;
      end loop;

      Status := HAL.I2C.Ok;
   end Wait_While_Flag;

   ----------------------
   -- Wait_Master_Flag --
   ----------------------

   procedure Wait_Master_Flag
     (This    : in out I2C_Port;
      Flag    : I2C_Status_Flag;
      Timeout : Natural;
      Status  : out HAL.I2C.I2C_Status)
   is
      Deadline : constant Time := Clock + Milliseconds (Timeout);
   begin
      while not Flag_Status (This, Flag) loop
         if This.Periph.ISR.NACKF then
            --  Generate STOP
            This.Periph.CR2.STOP := True;

            --  Clear the AF flag
            Clear_Flag (This, Ack_Failure);
            This.State := Ready;
            Status := HAL.I2C.Err_Error;

            return;
         end if;

         if Clock > Deadline then
            This.State := Ready;
            Status := HAL.I2C.Err_Timeout;

            return;
         end if;
      end loop;

      Status := HAL.I2C.Ok;
   end Wait_Master_Flag;

   --------------------------
   -- Master_Request_Write --
   --------------------------

   procedure Master_Request_Write
     (This    : in out I2C_Port;
      Addr    : HAL.I2C.I2C_Address;
      Nbytes  : Natural;
      Timeout : Natural;
      Status  : out HAL.I2C.I2C_Status) is
   begin
      -- Set number of bytes to send
      if Nbytes > 255 then
         This.Periph.CR2.NBYTES := 16#FF#;
         This.Periph.CR2.RELOAD := True;
      else
         This.Periph.CR2.NBYTES := HAL.UInt8 (NBytes);
         This.Periph.CR2.RELOAD := False;
      end if;
      -- Set Auto-End Mode for sending stop after N bytes are sent
      This.Periph.CR2.AUTOEND := True;
      -- Setup address and direction
      This.Periph.CR2.ADD10 :=
        This.Config.Addressing_Mode = Addressing_Mode_10bit;
      This.Periph.CR2.SADD := HAL.UInt10 (Addr);
      This.Periph.CR2.RD_WRN := False;
      -- Setup start bit
      This.Periph.CR2.START := True;
      -- Wait for address match flag
      Wait_Master_Flag (This, Address_Match, Timeout, Status);
   end Master_Request_Write;

   --------------------------
   -- Master_Request_Write --
   --------------------------

   procedure Master_Request_Read
     (This    : in out I2C_Port;
      Addr    : HAL.I2C.I2C_Address;
      Nbytes  : Natural;
      AutoEnd : Boolean;
      Timeout : Natural;
      Status  : out HAL.I2C.I2C_Status) is
   begin
      -- Read data
      This.Periph.CR2.RD_WRN := True;

      -- You need to give people their freedom, Lucas !!!
      if Nbytes > 255 then
         This.Periph.CR2.AUTOEND := False;
         This.Periph.CR2.RELOAD := True;
         This.Periph.CR2.NBYTES := 16#FF#;
      else
         This.Periph.CR2.AUTOEND := AutoEnd;
         This.Periph.CR2.RELOAD := False;
         This.Periph.CR2.NBYTES := HAL.UInt8 (Nbytes);
      end if;

      This.Periph.CR2.ADD10 :=
        This.Config.Addressing_Mode = Addressing_Mode_10bit;
      This.Periph.CR2.SADD := HAL.UInt10 (Addr);

      if Status /= HAL.I2C.Ok then
         return;
      end if;

      This.Periph.CR2.START := True;

      Wait_Master_Flag (This, Address_Match, Timeout, Status);
   end Master_Request_Read;

   -----------------------
   -- Mem_Request_Write --
   -----------------------

   procedure Mem_Request_Write
     (This          : in out I2C_Port;
      Addr          : HAL.I2C.I2C_Address;
      Mem_Addr      : UInt16;
      Mem_Addr_Size : HAL.I2C.I2C_Memory_Address_Size;
      Nbytes        : Natural;
      Timeout       : Natural;
      Status        : out HAL.I2C.I2C_Status) is
   begin
      -- Setup address and direction
      This.Periph.CR2.ADD10 :=
        This.Config.Addressing_Mode = Addressing_Mode_10bit;
      This.Periph.CR2.SADD := HAL.UInt10 (Addr);
      This.Periph.CR2.RD_WRN := False;
      This.Periph.CR2.NBYTES := 16#1#;
      This.Periph.CR2.RELOAD := True;
      -- Setup start bit
      This.Periph.CR2.START := True;
      -- Wait for address match flag
      Wait_Master_Flag (This, Address_Match, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         return;
      end if;
      -- Wait for TXIS flag to be set
      Wait_While_Flag (This, Tx_Interrupt_Status, True, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         return;
      end if;

      case Mem_Addr_Size is
         when HAL.I2C.Memory_Size_8b =>
            This.Periph.TXDR.TXDATA := UInt8 (Mem_Addr);

         when HAL.I2C.Memory_Size_16b =>
            This.Periph.TXDR.TXDATA := UInt8 (Shift_Right (Mem_Addr, 8));

            Wait_While_Flag (This, Tx_Complete_Reload, True, Timeout, Status);

            if Status /= HAL.I2C.Ok then
               return;
            end if;

            This.Periph.CR2.NBYTES := 16#1#;
            This.Periph.CR2.RELOAD := True;

            Wait_While_Flag (This, Tx_Interrupt_Status, True, Timeout, Status);

            if Status /= HAL.I2C.Ok then
               return;
            end if;

            This.Periph.TXDR.TXDATA := UInt8 (Mem_Addr and 16#FF#);
      end case;

      Wait_While_Flag (This, Tx_Complete_Reload, True, Timeout, Status);
      -- Set number of bytes to send
      if Nbytes > 255 then
         This.Periph.CR2.NBYTES := 16#FF#;
         This.Periph.CR2.RELOAD := True;
      else
         This.Periph.CR2.NBYTES := HAL.UInt8 (NBytes);
         This.Periph.CR2.RELOAD := False;
      end if;
      -- Set Auto-End Mode for sending stop after N bytes are sent
      This.Periph.CR2.AUTOEND := True;

   end Mem_Request_Write;

   ----------------------
   -- Mem_Request_Read --
   ----------------------

   procedure Mem_Request_Read
     (This          : in out I2C_Port;
      Addr          : HAL.I2C.I2C_Address;
      Mem_Addr      : UInt16;
      Mem_Addr_Size : HAL.I2C.I2C_Memory_Address_Size;
      Nbytes        : Natural;
      Timeout       : Natural;
      Status        : out HAL.I2C.I2C_Status) is
   begin
      This.Periph.CR2.ADD10 :=
        This.Config.Addressing_Mode = Addressing_Mode_10bit;
      This.Periph.CR2.SADD := UInt10 (Addr);
      This.Periph.CR2.NBYTES := 16#1#;
      This.Periph.CR2.RD_WRN := False;
      This.Periph.CR2.RELOAD := True;
      This.Periph.CR2.AUTOEND := False;

      This.Periph.CR2.START := True;

      Wait_Master_Flag (This, Address_Match, Timeout, Status);
      if Status /= HAL.I2C.Ok then
         return;
      end if;

      Wait_While_Flag (This, Tx_Interrupt_Status, True, Timeout, Status);
      if Status /= HAL.I2C.Ok then
         return;
      end if;

      case Mem_Addr_Size is
         when HAL.I2C.Memory_Size_8b =>
            This.Periph.TXDR.TXDATA := UInt8 (Mem_Addr);

         when HAL.I2C.Memory_Size_16b =>
            This.Periph.TXDR.TXDATA := UInt8 (Shift_Right (Mem_Addr, 8));

            Wait_While_Flag
              (This, Tx_Data_Register_Empty, False, Timeout, Status);

            if Status /= HAL.I2C.Ok then
               return;
            end if;

            This.Periph.CR2.NBYTES := 16#1#;

            Wait_While_Flag (This, Tx_Interrupt_Status, True, Timeout, Status);

            if Status /= HAL.I2C.Ok then
               return;
            end if;

            This.Periph.TXDR.TXDATA := UInt8 (Mem_Addr and 16#FF#);
      end case;

      Wait_While_Flag (This, Tx_Complete_Reload, True, Timeout, Status);
      This.Periph.CR2.STOP := True;
      This.Periph.CR2.RD_WRN := True;
      This.Periph.CR2.START := True;

      if Nbytes > 255 then
         This.Periph.CR2.NBYTES := 16#FF#;
         This.Periph.CR2.RELOAD := True;
      else
         This.Periph.CR2.NBYTES := HAL.UInt8 (NBytes);
         This.Periph.CR2.RELOAD := False;
      end if;
      -- Set Auto-End Mode for sending stop after N bytes are sent
      This.Periph.CR2.AUTOEND := True;
   end Mem_Request_Read;

   ---------------------
   -- Master_Transmit --
   ---------------------

   overriding
   procedure Master_Transmit
     (This    : in out I2C_Port;
      Addr    : HAL.I2C.I2C_Address;
      Data    : HAL.I2C.I2C_Data;
      Status  : out HAL.I2C.I2C_Status;
      Timeout : Natural := 1000) is
   begin
      if This.State = Reset then
         Status := HAL.I2C.Err_Error;
         return;

      elsif Data'Length = 0 then
         Status := HAL.I2C.Err_Error;
         return;
      end if;

      Wait_While_Flag (This, Busy, True, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         Status := HAL.I2C.Busy;
         return;
      end if;

      if This.State /= Ready then
         Status := HAL.I2C.Busy;
         return;
      end if;

      This.State := Master_Busy_Tx;

      Master_Request_Write (This, Addr, Data'Last, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         This.State := Reset;
         return;
      end if;

      Dispatch (This).Data_Send
        (Data => Data, Timeout => Timeout, Status => Status);
      This.State := Ready;
      Clear_Flag (This, Address_Match);
   end Master_Transmit;

   --------------------
   -- Master_Receive --
   --------------------

   overriding
   procedure Master_Receive
     (This    : in out I2C_Port;
      Addr    : HAL.I2C.I2C_Address;
      Data    : out HAL.I2C.I2C_Data;
      Status  : out HAL.I2C.I2C_Status;
      Timeout : Natural := 1000) is
   begin
      if This.State = Reset then
         Status := HAL.I2C.Err_Error;
         return;

      elsif Data'Length = 0 then
         Status := HAL.I2C.Err_Error;
         return;
      end if;

      Wait_While_Flag (This, Busy, True, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         Status := HAL.I2C.Busy;
         return;
      end if;

      if This.State /= Ready then
         Status := HAL.I2C.Busy;
         return;
      end if;

      This.State := Master_Busy_Rx;

      Master_Request_Read (This, Addr, Data'Last, True, Timeout, Status);

      --  Use a dispatching call in case Data_Receive is overridden for DMA
      --  transfert.
      Dispatch (This).Data_Receive (Data, Timeout, Status);

      This.State := Ready;
   end Master_Receive;

   ---------------
   -- Mem_Write --
   ---------------

   overriding
   procedure Mem_Write
     (This          : in out I2C_Port;
      Addr          : HAL.I2C.I2C_Address;
      Mem_Addr      : UInt16;
      Mem_Addr_Size : HAL.I2C.I2C_Memory_Address_Size;
      Data          : HAL.I2C.I2C_Data;
      Status        : out HAL.I2C.I2C_Status;
      Timeout       : Natural := 1000) is
   begin
      if This.State = Reset then
         Status := HAL.I2C.Err_Error;
         return;

      elsif Data'Length = 0 then
         Status := HAL.I2C.Err_Error;
         return;
      end if;

      Wait_While_Flag (This, Busy, True, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         Status := HAL.I2C.Busy;
         return;
      end if;

      if This.State /= Ready then
         Status := HAL.I2C.Busy;
         return;
      end if;

      This.State := Mem_Busy_Tx;

      Mem_Request_Write (This, Addr, Mem_Addr, Mem_Addr_Size, Data'Last, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         return;
      end if;

      --  Use a dispatching call in case Data_Send is overridden for DMA
      --  transfert.
      Dispatch (This).Data_Send
        (Data => Data, Timeout => Timeout, Status => Status);

      if Status /= HAL.I2C.Ok then
         return;
      end if;

      This.State := Ready;
   end Mem_Write;

   --------------
   -- Mem_Read --
   --------------

   overriding
   procedure Mem_Read
     (This          : in out I2C_Port;
      Addr          : HAL.I2C.I2C_Address;
      Mem_Addr      : UInt16;
      Mem_Addr_Size : HAL.I2C.I2C_Memory_Address_Size;
      Data          : out HAL.I2C.I2C_Data;
      Status        : out HAL.I2C.I2C_Status;
      Timeout       : Natural := 1000) is
   begin
      if This.State = Reset then
         Status := HAL.I2C.Err_Error;
         return;

      elsif Data'Length = 0 then
         Status := HAL.I2C.Err_Error;
         return;
      end if;

      Wait_While_Flag (This, Busy, True, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         Status := HAL.I2C.Busy;
         return;
      end if;

      if This.State /= Ready then
         Status := HAL.I2C.Busy;
         return;
      end if;

      This.State := Mem_Busy_Rx;

      Mem_Request_Read (This, Addr, Mem_Addr, Mem_Addr_Size, Data'Last, Timeout, Status);

      if Status /= HAL.I2C.Ok then
         return;
      end if;

      if Data'Length = 1 then
         --  Disable acknowledge
         This.Periph.CR2.NACK := False;
         This.Periph.CR2.STOP := True;
      end if;

      --  Use a dispatching call in case Data_Receive is overridden for DMA
      --  transfer.
      Dispatch (This).Data_Receive (Data, Timeout, Status);

      This.State := Ready;
   end Mem_Read;

   ---------------
   -- Set_State --
   ---------------

   procedure Set_State (This : in out I2C_Port; Enabled : Boolean) is
   begin
      This.Periph.CR1.PE := Enabled;
   end Set_State;

   ------------------
   -- Port_Enabled --
   ------------------

   function Port_Enabled (This : I2C_Port) return Boolean is
   begin
      return This.Periph.CR1.PE;
   end Port_Enabled;

   ----------------------
   -- Enable_Interrupt --
   ----------------------

   procedure Enable_Interrupt (This : in out I2C_Port; Source : I2C_Interrupt_Type)
   is
   begin
      case Source is
         when Error =>
            This.Periph.CR1.ERRIE := True;

         when TransferComplete =>
            This.Periph.CR1.TCIE := True;

         when Receive =>
            This.Periph.CR1.RXIE := True;

         when Transmit =>
            This.Periph.CR1.TXIE := True;

         when Stop =>
            This.Periph.CR1.STOPIE := True;

         when NotAcknowledge =>
            This.Periph.CR1.NACKIE := True;

         when AddressMatch =>
            This.Periph.CR1.ADDRIE := True;

      end case;
   end Enable_Interrupt;

   -----------------------
   -- Disable_Interrupt --
   -----------------------

   procedure Disable_Interrupt (This : in out I2C_Port; Source : I2C_Interrupt_Type)
   is
   begin
      case Source is
         when Error =>
            This.Periph.CR1.ERRIE := False;

         when TransferComplete =>
            This.Periph.CR1.TCIE := False;

         when Receive =>
            This.Periph.CR1.RXIE := False;

         when Transmit =>
            This.Periph.CR1.TXIE := False;

         when Stop =>
            This.Periph.CR1.STOPIE := False;

         when NotAcknowledge =>
            This.Periph.CR1.NACKIE := False;

         when AddressMatch =>
            This.Periph.CR1.ADDRIE := False;

      end case;
   end Disable_Interrupt;

   -------------
   -- Enabled --
   -------------

   function Enabled
     (This : I2C_Port; Source : I2C_Interrupt_Type) return Boolean is
   begin
      case Source is
         when Error =>
            return This.Periph.CR1.ERRIE;

         when TransferComplete =>
            return This.Periph.CR1.TCIE;

         when Receive =>
            return This.Periph.CR1.RXIE;

         when Transmit =>
            return This.Periph.CR1.TXIE;

         when Stop =>
            return This.Periph.CR1.STOPIE;

         when NotAcknowledge =>
            return This.Periph.CR1.NACKIE;

         when AddressMatch =>
            return This.Periph.CR1.ADDRIE;

      end case;
   end Enabled;

   ---------------------------
   -- Data_Register_Address --
   ---------------------------

   function TX_Data_Register_Address (This : I2C_Port) return System.Address
   is (This.Periph.TXDR'Address);

   function RX_Data_Register_Address (This : I2C_Port) return System.Address
   is (This.Periph.RXDR'Address);
   -------------------
   -- Data_Transfer --
   -------------------

   procedure Data_Send
     (This    : in out I2C_Port;
      Data    : HAL.I2C.I2C_Data;
      Timeout : Natural;
      Status  : out HAL.I2C.I2C_Status)
   is
      Idx : Natural := Data'First;
      N   : Natural := Data'Last;
   begin
    Send_loop:
      loop
         Wait_While_Flag (This, Tx_Interrupt_Status, False, Timeout, Status);

         if Status /= HAL.I2C.Ok then
            return;
         end if;

         This.Periph.TXDR.TXDATA := Data (Idx);
         Idx := Idx + 1;

         if not Flag_Status (This, Tx_Complete) then
            if not Flag_Status (This, Tx_Complete_Reload) then
               if N < 256 then
                  This.Periph.CR2.NBYTES := HAL.UInt8 (N);
                  N := 0;
                  This.Periph.CR2.RELOAD := False;
               else
                  N := N - 255;
                  This.Periph.CR2.NBYTES := 16#FF#;
                  This.Periph.CR2.RELOAD := True;
               end if;
            else
               exit Send_loop;
            end if;
         end if;
      end loop Send_loop;
   end Data_Send;

   ------------------
   -- Data_Receive --
   ------------------

   procedure Data_Receive
     (This    : in out I2C_Port;
      Data    : out HAL.I2C.I2C_Data;
      Timeout : Natural;
      Status  : out HAL.I2C.I2C_Status)
   is
      N   : Natural := Data'Last;
      Idx : Natural := Data'First;
   begin
     Receive_loop:
      loop
         Wait_While_Flag
           (This, Rx_Data_Register_Not_Empty, False, Timeout, Status);
         if Status /= HAL.I2C.Ok then
            return;
         end if;

         Data (Idx) := This.Periph.RXDR.RXDATA;
         Idx := Idx + 1;

         if (This.Periph.CR2.NBYTES > 0) then
            This.Periph.CR2.NBYTES := This.Periph.CR2.NBYTES - 1;
         else
            if Flag_Status (This, Rx_Data_Register_Not_Empty) then
               if Flag_Status (This, Stop_Detection) then
                  if N > 256 then
                     This.Periph.CR2.NBYTES := 16#FF#;
                     This.Periph.CR2.RELOAD := True;
                     N := N - 255;
                  else
                     This.Periph.CR2.NBYTES := HAL.UInt8 (N);
                     This.Periph.CR2.RELOAD := False;
                     N := 0;
                     -- FIXME
                  end if;
               else
                  exit Receive_loop;
               end if;
            else
               exit Receive_loop; -- FIXME
            end if;
         end if;

      end loop Receive_loop;
   end Data_Receive;

end STM32.I2C;
