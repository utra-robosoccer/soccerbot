# LwIP + FreeRTOS CubeMX configuration for a working Netconn appliction

## Board

STM32F767ZI

## Configuration changes

The following patch shows all the options that changed from a blank project initialized from its default settings and only enables Lwip and FreeRTOS. The MPU, flash settings, LwIP, and FreeRTOS need significant configuration changes.

```
diff --git a/./STM32Cube/blank_lwip_rtos/blank_lwip_rtos.ioc b/./soccer-embedded/Development/Ethernet/lwip-rtos-config/lwip-rtos-config/lwip-rtos-config.ioc
index 8edd3a1..641b5e6 100644
--- a/./STM32Cube/blank_lwip_rtos/blank_lwip_rtos.ioc
+++ b/./soccer-embedded/Development/Ethernet/lwip-rtos-config/lwip-rtos-config/lwip-rtos-config.ioc
@@ -1,13 +1,62 @@
 #MicroXplorer Configuration settings - do not modify
+CORTEX_M7.ART_ACCLERATOR_ENABLE=1
+CORTEX_M7.AccessPermission-Cortex_Memory_Protection_Unit_Region0_Settings=MPU_REGION_FULL_ACCESS
+CORTEX_M7.AccessPermission-Cortex_Memory_Protection_Unit_Region1_Settings=MPU_REGION_FULL_ACCESS
+CORTEX_M7.AccessPermission-Cortex_Memory_Protection_Unit_Region2_Settings=MPU_REGION_FULL_ACCESS
+CORTEX_M7.BaseAddress-Cortex_Memory_Protection_Unit_Region0_Settings=0x20020000
+CORTEX_M7.BaseAddress-Cortex_Memory_Protection_Unit_Region1_Settings=0x2007C000
+CORTEX_M7.BaseAddress-Cortex_Memory_Protection_Unit_Region2_Settings=0x2007C000
+CORTEX_M7.CPU_DCache=Enabled
+CORTEX_M7.CPU_ICache=Enabled
+CORTEX_M7.Enable-Cortex_Memory_Protection_Unit_Region0_Settings=MPU_REGION_ENABLE
+CORTEX_M7.Enable-Cortex_Memory_Protection_Unit_Region1_Settings=MPU_REGION_ENABLE
+CORTEX_M7.Enable-Cortex_Memory_Protection_Unit_Region2_Settings=MPU_REGION_ENABLE
+CORTEX_M7.IPParameters=CPU_ICache,CPU_DCache,MPU_Control,Enable-Cortex_Memory_Protection_Unit_Region0_Settings,BaseAddress-Cortex_Memory_Protection_Unit_Region0_Settings,Size-Cortex_Memory_Protection_Unit_Region0_Settings,AccessPermission-Cortex_Memory_Protection_Unit_Region0_Settings,IsCacheable-Cortex_Memory_Protection_Unit_Region0_Settings,Enable-Cortex_Memory_Protection_Unit_Region1_Settings,BaseAddress-Cortex_Memory_Protection_Unit_Region1_Settings,Size-Cortex_Memory_Protection_Unit_Region1_Settings,TypeExtField-Cortex_Memory_Protection_Unit_Region1_Settings,AccessPermission-Cortex_Memory_Protection_Unit_Region1_Settings,IsShareable-Cortex_Memory_Protection_Unit_Region1_Settings,Enable-Cortex_Memory_Protection_Unit_Region2_Settings,BaseAddress-Cortex_Memory_Protection_Unit_Region2_Settings,Size-Cortex_Memory_Protection_Unit_Region2_Settings,AccessPermission-Cortex_Memory_Protection_Unit_Region2_Settings,IsShareable-Cortex_Memory_Protection_Unit_Region2_Settings,IsBufferable-Cortex_Memory_Protection_Unit_Region2_Settings,ART_ACCLERATOR_ENABLE,PREFETCH_ENABLE
+CORTEX_M7.IsBufferable-Cortex_Memory_Protection_Unit_Region2_Settings=MPU_ACCESS_BUFFERABLE
+CORTEX_M7.IsCacheable-Cortex_Memory_Protection_Unit_Region0_Settings=MPU_ACCESS_CACHEABLE
+CORTEX_M7.IsShareable-Cortex_Memory_Protection_Unit_Region1_Settings=MPU_ACCESS_SHAREABLE
+CORTEX_M7.IsShareable-Cortex_Memory_Protection_Unit_Region2_Settings=MPU_ACCESS_SHAREABLE
+CORTEX_M7.MPU_Control=MPU_PRIVILEGED_DEFAULT
+CORTEX_M7.PREFETCH_ENABLE=1
+CORTEX_M7.Size-Cortex_Memory_Protection_Unit_Region0_Settings=MPU_REGION_SIZE_512KB
+CORTEX_M7.Size-Cortex_Memory_Protection_Unit_Region1_Settings=MPU_REGION_SIZE_16KB
+CORTEX_M7.Size-Cortex_Memory_Protection_Unit_Region2_Settings=MPU_REGION_SIZE_256B
+CORTEX_M7.TypeExtField-Cortex_Memory_Protection_Unit_Region1_Settings=MPU_TEX_LEVEL1
 ETH.IPParameters=MediaInterface,PHY_Name,PHY_Value,PhyAddress
 ETH.MediaInterface=ETH_MEDIA_INTERFACE_RMII
 ETH.PHY_Name=LAN8742A_PHY_ADDRESS
 ETH.PHY_Value=0
 ETH.PhyAddress=0
-FREERTOS.IPParameters=Tasks01
+FREERTOS.FootprintOK=true
+FREERTOS.INCLUDE_vTaskSuspend=0
+FREERTOS.INCLUDE_xTaskResumeFromISR=0
+FREERTOS.IPParameters=Tasks01,configUSE_COUNTING_SEMAPHORES,configUSE_RECURSIVE_MUTEXES,configTOTAL_HEAP_SIZE,FootprintOK,configUSE_TRACE_FACILITY,configUSE_STATS_FORMATTING_FUNCTIONS,INCLUDE_xTaskResumeFromISR,INCLUDE_vTaskSuspend
 FREERTOS.Tasks01=defaultTask,0,128,StartDefaultTask,Default,NULL
+FREERTOS.configTOTAL_HEAP_SIZE=25600
+FREERTOS.configUSE_COUNTING_SEMAPHORES=1
+FREERTOS.configUSE_RECURSIVE_MUTEXES=1
+FREERTOS.configUSE_STATS_FORMATTING_FUNCTIONS=1
+FREERTOS.configUSE_TRACE_FACILITY=1
 File.Version=6
 KeepUserPlacement=true
+LWIP.CHECKSUM_BY_HARDWARE=1
+LWIP.HTTPD_USE_CUSTOM_FSDATA=1
+LWIP.IPParameters=MEMP_NUM_TCP_PCB,MEMP_NUM_UDP_PCB,PBUF_POOL_BUFSIZE,MEMP_NUM_SYS_TIMEOUT,MEMP_NUM_TCP_PCB_LISTEN,MEMP_NUM_PBUF,MEM_SIZE,CHECKSUM_BY_HARDWARE,TCP_QUEUE_OOSEQ,TCP_MSS,TCP_SND_BUF,TCP_SND_QUEUELEN,TCP_WND,LWIP_NETIF_LINK_CALLBACK,LWIP_SOCKET,LWIP_HTTPD,HTTPD_USE_CUSTOM_FSDATA
+LWIP.LWIP_HTTPD=1
+LWIP.LWIP_NETIF_LINK_CALLBACK=1
+LWIP.LWIP_SOCKET=0
+LWIP.MEMP_NUM_PBUF=10
+LWIP.MEMP_NUM_SYS_TIMEOUT=10
+LWIP.MEMP_NUM_TCP_PCB=10
+LWIP.MEMP_NUM_TCP_PCB_LISTEN=5
+LWIP.MEMP_NUM_UDP_PCB=6
+LWIP.MEM_SIZE=10240
+LWIP.PBUF_POOL_BUFSIZE=1524
+LWIP.TCP_MSS=1460
+LWIP.TCP_QUEUE_OOSEQ=0
+LWIP.TCP_SND_BUF=5840
+LWIP.TCP_SND_QUEUELEN=8
+LWIP.TCP_WND=2920
 LWIP.Version=v2.0.3_Cube
 Mcu.Family=STM32F7
 Mcu.IP0=CORTEX_M7
@@ -46,7 +95,7 @@ Mcu.Pin27=PB7
 Mcu.Pin28=VP_FREERTOS_VS_ENABLE
 Mcu.Pin29=VP_LWIP_VS_Enabled
 Mcu.Pin3=PH0/OSC_IN
-Mcu.Pin30=VP_SYS_VS_Systick
+Mcu.Pin30=VP_SYS_VS_tim1
 Mcu.Pin4=PH1/OSC_OUT
 Mcu.Pin5=PC1
 Mcu.Pin6=PA1
@@ -59,17 +108,20 @@ Mcu.UserConstants=
 Mcu.UserName=STM32F767ZITx
 MxCube.Version=4.26.1
 MxDb.Version=DB.4.0.261
-NVIC.BusFault_IRQn=true\:0\:0\:false\:false\:false\:false\:true
-NVIC.DebugMonitor_IRQn=true\:0\:0\:false\:false\:false\:false\:true
+NVIC.BusFault_IRQn=true\:0\:0\:false\:false\:true\:false\:true
+NVIC.DebugMonitor_IRQn=true\:0\:0\:false\:false\:true\:false\:true
 NVIC.ETH_IRQn=true\:5\:0\:false\:false\:true\:true\:false
-NVIC.HardFault_IRQn=true\:0\:0\:false\:false\:false\:false\:true
-NVIC.MemoryManagement_IRQn=true\:0\:0\:false\:false\:false\:false\:true
-NVIC.NonMaskableInt_IRQn=true\:0\:0\:false\:false\:false\:false\:true
+NVIC.HardFault_IRQn=true\:0\:0\:false\:false\:true\:false\:true
+NVIC.MemoryManagement_IRQn=true\:0\:0\:false\:false\:true\:false\:true
+NVIC.NonMaskableInt_IRQn=true\:0\:0\:false\:false\:true\:false\:true
 NVIC.PendSV_IRQn=true\:15\:0\:false\:false\:false\:true\:true
 NVIC.PriorityGroup=NVIC_PRIORITYGROUP_4
 NVIC.SVCall_IRQn=true\:0\:0\:false\:false\:false\:false\:true
 NVIC.SysTick_IRQn=true\:15\:0\:false\:false\:true\:true\:true
-NVIC.UsageFault_IRQn=true\:0\:0\:false\:false\:false\:false\:true
+NVIC.TIM1_UP_TIM10_IRQn=true\:0\:0\:false\:false\:true\:false\:false
+NVIC.TimeBase=TIM1_UP_TIM10_IRQn
+NVIC.TimeBaseIP=TIM1
+NVIC.UsageFault_IRQn=true\:0\:0\:false\:false\:true\:false\:true
 PA1.GPIOParameters=GPIO_Label
 PA1.GPIO_Label=RMII_REF_CLK [LAN8742A-CZ-TR_REFCLK0]
 PA1.Locked=true
@@ -206,6 +258,32 @@ PH1/OSC_OUT.Locked=true
 PH1/OSC_OUT.Mode=HSE-External-Clock-Source
 PH1/OSC_OUT.Signal=RCC_OSC_OUT
 PinOutPanel.RotationAngle=0
+ProjectManager.AskForMigrate=true
+ProjectManager.BackupPrevious=false
+ProjectManager.CompilerOptimize=6
+ProjectManager.ComputerToolchain=false
+ProjectManager.CoupleFile=true
+ProjectManager.CustomerFirmwarePackage=
+ProjectManager.DefaultFWLocation=true
+ProjectManager.DeletePrevious=true
+ProjectManager.DeviceId=STM32F767ZITx
+ProjectManager.FirmwarePackage=STM32Cube FW_F7 V1.12.0
+ProjectManager.FreePins=false
+ProjectManager.HalAssertFull=false
+ProjectManager.HeapSize=0x200
+ProjectManager.KeepUserCode=true
+ProjectManager.LastFirmware=true
+ProjectManager.LibraryCopy=1
+ProjectManager.MainLocation=Src
+ProjectManager.PreviousToolchain=SW4STM32
+ProjectManager.ProjectBuild=false
+ProjectManager.ProjectFileName=lwip-rtos-config.ioc
+ProjectManager.ProjectName=lwip-rtos-config
+ProjectManager.StackSize=0x400
+ProjectManager.TargetToolchain=SW4STM32
+ProjectManager.ToolChainLocation=
+ProjectManager.UnderRoot=true
+ProjectManager.functionlistsort=1-MX_GPIO_Init-GPIO-false-HAL-true,2-SystemClock_Config-RCC-false-HAL-false,3-MX_CORTEX_M7_Init-CORTEX_M7-false-HAL-true,4-MX_USART3_UART_Init-USART3-false-HAL-true,5-MX_USB_OTG_FS_PCD_Init-USB_OTG_FS-false-HAL-true,6-MX_LWIP_Init-LWIP-false-HAL-true
 RCC.48MHZClocksFreq_Value=24000000
 RCC.ADC12outputFreq_Value=72000000
 RCC.ADC34outputFreq_Value=72000000
@@ -302,7 +380,7 @@ VP_FREERTOS_VS_ENABLE.Mode=Enabled
 VP_FREERTOS_VS_ENABLE.Signal=FREERTOS_VS_ENABLE
 VP_LWIP_VS_Enabled.Mode=Enabled
 VP_LWIP_VS_Enabled.Signal=LWIP_VS_Enabled
-VP_SYS_VS_Systick.Mode=SysTick
-VP_SYS_VS_Systick.Signal=SYS_VS_Systick
+VP_SYS_VS_tim1.Mode=TIM1
+VP_SYS_VS_tim1.Signal=SYS_VS_tim1
 board=NUCLEO-F767ZI
 boardIOC=true

```
