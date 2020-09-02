#ifndef __TCPIP_H
#define __TCPIP_H

#include <stdint.h>
#include <stdarg.h>
#include <time.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>

#include <libopencm3/ethernet/mac.h>
#include <libopencm3/ethernet/phy_lan87xx.h>

#define MAX_USART_RX_LEN	128
#define MAX_ARGS		10
#define MAGIC_55		0x55
#define USART_LOOP		10000
#define IRQ2NVIC_PRIOR(x)	((x)<<4)

struct cmd_args {
	char ptr[MAX_USART_RX_LEN];
	char *argv[MAX_ARGS];
	int argc, timer_id, interval;
};

typedef void (*gosh_cmd_handler)(char **, int);

/* USART */
void usart_send_hex(uint32_t, uint8_t *, int, int, int);
void setup_usart_speed(uint32_t, uint32_t, int);
void append_usart_rx_queue(uint32_t, char);
void setup_gosh_callback(gosh_cmd_handler);
void generic_usart_handler(void *);

int std_printf(const char *format, ...);

int usart_timeout_putc(uint32_t, char);
void console_puts(char *);
void console_putc(char);
void hex_dump(char *, int);

void start_usart_task(void);
void setup_usart3(void);

#define lUDPLoggingPrintf(fmt, args...) do {;} while (0)

#ifndef SYSCFG_PMC
#define SYSCFG_PMC MMIO32(SYSCFG_BASE + 0x04)
#define AFIO_MAPR_MII_RMII_SEL	(1 << 23)
#endif

/* some structure definitions copy from STM32 HAL Library */

typedef enum
{
	HAL_OK       = 0x00U,
	HAL_ERROR    = 0x01U,
	HAL_BUSY     = 0x02U,
	HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum
{
	HAL_ETH_STATE_RESET             = 0x00U,    /*!< Peripheral not yet Initialized or disabled         */
	HAL_ETH_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use           */
	HAL_ETH_STATE_BUSY              = 0x02U,    /*!< an internal process is ongoing                     */
	HAL_ETH_STATE_BUSY_TX           = 0x12U,    /*!< Data Transmission process is ongoing               */
	HAL_ETH_STATE_BUSY_RX           = 0x22U,    /*!< Data Reception process is ongoing                  */
	HAL_ETH_STATE_BUSY_TX_RX        = 0x32U,    /*!< Data Transmission and Reception process is ongoing */
	HAL_ETH_STATE_BUSY_WR           = 0x42U,    /*!< Write process is ongoing                           */
	HAL_ETH_STATE_BUSY_RD           = 0x82U,    /*!< Read process is ongoing                            */
	HAL_ETH_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                                      */
	HAL_ETH_STATE_ERROR             = 0x04U     /*!< Reception process is ongoing                       */
} HAL_ETH_StateTypeDef;

typedef enum
{
	HAL_UNLOCKED = 0x00U,
	HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

typedef struct
{
	/*!< Selects or not the AutoNegotiation mode for the external PHY.
	 * The AutoNegotiation allows an automatic setting of the Speed (10/100Mbps) and the mode (half/full-duplex).
	 * This parameter can be a value of @ref ETH_AutoNegotiation
	 */
	uint32_t AutoNegotiation;

	/*!< Sets the Ethernet speed: 10/100 Mbps.
	 * This parameter can be a value of @ref ETH_Speed
	 */
	uint32_t Speed;

	/*!< Selects the MAC duplex mode: Half-Duplex or Full-Duplex mode
	 * This parameter can be a value of @ref ETH_Duplex_Mode
	 */
	uint32_t DuplexMode;

	/*!< Ethernet PHY address.  This parameter must be a number between Min_Data = 0 and Max_Data = 32 */
	uint16_t PhyAddress;

	/*!< MAC Address of used Hardware: must be pointer on an array of 6 bytes */
	uint8_t *MACAddr;

	/*!< Selects the Ethernet Rx mode: Polling mode, Interrupt mode.  This parameter can be a value of @ref ETH_Rx_Mode */
	uint32_t RxMode;

	/*!< Selects if the checksum is check by hardware or by software. This parameter can be a value of @ref ETH_Checksum_Mode */
	uint32_t ChecksumMode;

	/*!< Selects the media-independent interface or the reduced media-independent interface. This parameter can be a value of @ref ETH_Media_Interface */
	uint32_t MediaInterface;
} ETH_InitTypeDef;

typedef struct
{
	/*!< Selects or not the Watchdog timer. When enabled, the MAC allows no more then 2048 bytes to be received.  When disabled, the MAC can receive up to 16384 bytes.  This parameter can be a value of @ref ETH_Watchdog */
	uint32_t Watchdog;

	/*!< Selects or not Jabber timer. When enabled, the MAC allows no more then 2048 bytes to be sent.  When disabled, the MAC can send up to 16384 bytes.  This parameter can be a value of @ref ETH_Jabber */
	uint32_t Jabber;

	/*!< Selects the minimum IFG between frames during transmission. This parameter can be a value of @ref ETH_Inter_Frame_Gap */
	uint32_t InterFrameGap;

	/*!< Selects or not the Carrier Sense. This parameter can be a value of @ref ETH_Carrier_Sense */
	uint32_t CarrierSense;

	/*!< Selects or not the ReceiveOwn, ReceiveOwn allows the reception of frames when the TX_EN signal is asserted in Half-Duplex mode.  This parameter can be a value of @ref ETH_Receive_Own */
	uint32_t ReceiveOwn;

	/*!< Selects or not the internal MAC MII Loopback mode.  This parameter can be a value of @ref ETH_Loop_Back_Mode */
	uint32_t LoopbackMode;

	/*!< Selects or not the IPv4 checksum checking for received frame payloads' TCP/UDP/ICMP headers. This parameter can be a value of @ref ETH_Checksum_Offload */
	uint32_t ChecksumOffload;

	/*!< Selects or not the MAC attempt retries transmission, based on the settings of BL, when a collision occurs (Half-Duplex mode).  This parameter can be a value of @ref ETH_Retry_Transmission */
	uint32_t RetryTransmission;

	/*!< Selects or not the Automatic MAC Pad/CRC Stripping. This parameter can be a value of @ref ETH_Automatic_Pad_CRC_Strip */
	uint32_t AutomaticPadCRCStrip;

	/*!< Selects the BackOff limit value. This parameter can be a value of @ref ETH_Back_Off_Limit */
	uint32_t BackOffLimit;

	/*!< Selects or not the deferral check function (Half-Duplex mode). This parameter can be a value of @ref ETH_Deferral_Check */
	uint32_t DeferralCheck;

	/*!< Selects or not all frames reception by the MAC (No filtering). This parameter can be a value of @ref ETH_Receive_All */
	uint32_t ReceiveAll;

	/*!< Selects the Source Address Filter mode. This parameter can be a value of @ref ETH_Source_Addr_Filter */
	uint32_t SourceAddrFilter;

	/*!< Sets the forwarding mode of the control frames (including unicast and multicast PAUSE frames). This parameter can be a value of @ref ETH_Pass_Control_Frames */
	uint32_t PassControlFrames;

	/*!< Selects or not the reception of Broadcast Frames. This parameter can be a value of @ref ETH_Broadcast_Frames_Reception */
	uint32_t BroadcastFramesReception;

	/*!< Sets the destination filter mode for both unicast and multicast frames. This parameter can be a value of @ref ETH_Destination_Addr_Filter */
	uint32_t DestinationAddrFilter;

	/*!< Selects or not the Promiscuous Mode. This parameter can be a value of @ref ETH_Promiscuous_Mode */
	uint32_t PromiscuousMode;

	/*!< Selects the Multicast Frames filter mode: None/HashTableFilter/PerfectFilter/PerfectHashTableFilter. This parameter can be a value of @ref ETH_Multicast_Frames_Filter */
	uint32_t MulticastFramesFilter;

	/*!< Selects the Unicast Frames filter mode: HashTableFilter/PerfectFilter/PerfectHashTableFilter. This parameter can be a value of @ref ETH_Unicast_Frames_Filter */
	uint32_t UnicastFramesFilter;

	/*!< This field holds the higher 32 bits of Hash table. This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xFFFFFFFF */
	uint32_t HashTableHigh;

	/*!< This field holds the lower 32 bits of Hash table. This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xFFFFFFFF  */
	uint32_t HashTableLow;

	/*!< This field holds the value to be used in the Pause Time field in the transmit control frame. This parameter must be a number between Min_Data = 0x0 and Max_Data = 0xFFFF */
	uint32_t PauseTime;

	/*!< Selects or not the automatic generation of Zero-Quanta Pause Control frames. This parameter can be a value of @ref ETH_Zero_Quanta_Pause */
	uint32_t ZeroQuantaPause;

	/*!< This field configures the threshold of the PAUSE to be checked for automatic retransmission of PAUSE Frame.  This parameter can be a value of @ref ETH_Pause_Low_Threshold */
	uint32_t PauseLowThreshold;

	/*!< Selects or not the MAC detection of the Pause frames (with MAC Address0 unicast address and unique multicast address). This parameter can be a value of @ref ETH_Unicast_Pause_Frame_Detect */
	uint32_t UnicastPauseFrameDetect;

	/*!< Enables or disables the MAC to decode the received Pause frame and disable its transmitter for a specified time (Pause Time) This parameter can be a value of @ref ETH_Receive_Flow_Control */
	uint32_t ReceiveFlowControl;

	/*!< Enables or disables the MAC to transmit Pause frames (Full-Duplex mode) or the MAC back-pressure operation (Half-Duplex mode) This parameter can be a value of @ref ETH_Transmit_Flow_Control */
	uint32_t TransmitFlowControl;

	/*!< Selects the 12-bit VLAN identifier or the complete 16-bit VLAN tag for comparison and filtering. This parameter can be a value of @ref ETH_VLAN_Tag_Comparison */
	uint32_t VLANTagComparison;

	/*!< Holds the VLAN tag identifier for receive frames */
	uint32_t VLANTagIdentifier;
} ETH_MACInitTypeDef;

typedef struct
{
	uint32_t DropTCPIPChecksumErrorFrame; /*!< Selects or not the Dropping of TCP/IP Checksum Error Frames. This parameter can be a value of @ref ETH_Drop_TCP_IP_Checksum_Error_Frame */
	uint32_t ReceiveStoreForward;         /*!< Enables or disables the Receive store and forward mode. This parameter can be a value of @ref ETH_Receive_Store_Forward */
	uint32_t FlushReceivedFrame;          /*!< Enables or disables the flushing of received frames. This parameter can be a value of @ref ETH_Flush_Received_Frame */
	uint32_t TransmitStoreForward;        /*!< Enables or disables Transmit store and forward mode. This parameter can be a value of @ref ETH_Transmit_Store_Forward */
	uint32_t TransmitThresholdControl;    /*!< Selects or not the Transmit Threshold Control. This parameter can be a value of @ref ETH_Transmit_Threshold_Control */
	uint32_t ForwardErrorFrames;          /*!< Selects or not the forward to the DMA of erroneous frames. This parameter can be a value of @ref ETH_Forward_Error_Frames */
	uint32_t ForwardUndersizedGoodFrames; /*!< Enables or disables the Rx FIFO to forward Undersized frames (frames with no Error and length less than 64 bytes) including pad-bytes and CRC) This parameter can be a value of @ref ETH_Forward_Undersized_Good_Frames */
	uint32_t ReceiveThresholdControl;     /*!< Selects the threshold level of the Receive FIFO. This parameter can be a value of @ref ETH_Receive_Threshold_Control */
	uint32_t SecondFrameOperate;          /*!< Selects or not the Operate on second frame mode, which allows the DMA to process a second frame of Transmit data even before obtaining the status for the first frame.  This parameter can be a value of @ref ETH_Second_Frame_Operate */
	uint32_t AddressAlignedBeats;         /*!< Enables or disables the Address Aligned Beats. This parameter can be a value of @ref ETH_Address_Aligned_Beats */
	uint32_t FixedBurst;                  /*!< Enables or disables the AHB Master interface fixed burst transfers. This parameter can be a value of @ref ETH_Fixed_Burst */
	uint32_t RxDMABurstLength;            /*!< Indicates the maximum number of beats to be transferred in one Rx DMA transaction. This parameter can be a value of @ref ETH_Rx_DMA_Burst_Length */
	uint32_t TxDMABurstLength;            /*!< Indicates the maximum number of beats to be transferred in one Tx DMA transaction. This parameter can be a value of @ref ETH_Tx_DMA_Burst_Length */
	uint32_t EnhancedDescriptorFormat;    /*!< Enables the enhanced descriptor format. This parameter can be a value of @ref ETH_DMA_Enhanced_descriptor_format */
	uint32_t DescriptorSkipLength;        /*!< Specifies the number of word to skip between two unchained descriptors (Ring mode). This parameter must be a number between Min_Data = 0 and Max_Data = 32 */
	uint32_t DMAArbitration;              /*!< Selects the DMA Tx/Rx arbitration. This parameter can be a value of @ref ETH_DMA_Arbitration */
} ETH_DMAInitTypeDef;

typedef struct
{
	volatile uint32_t Status;           /*!< Status */
	uint32_t ControlBufferSize;     /*!< Control and Buffer1, Buffer2 lengths */
	uint32_t Buffer1Addr;           /*!< Buffer1 address pointer */
	uint32_t Buffer2NextDescAddr;   /*!< Buffer2 or next descriptor address pointer */

	/*!< Enhanced Ethernet DMA PTP Descriptors */
	uint32_t ExtendedStatus;        /*!< Extended status for PTP receive descriptor */
	uint32_t Reserved1;             /*!< Reserved */
	uint32_t TimeStampLow;          /*!< Time Stamp Low value for transmit and receive */
	uint32_t TimeStampHigh;         /*!< Time Stamp High value for transmit and receive */
} ETH_DMADescTypeDef;

typedef struct
{
	ETH_DMADescTypeDef *FSRxDesc;          /*!< First Segment Rx Desc */
	ETH_DMADescTypeDef *LSRxDesc;          /*!< Last Segment Rx Desc */
	uint32_t SegCount;                    /*!< Segment count */
	uint32_t length;                       /*!< Frame length */
	uint32_t buffer;                       /*!< Frame buffer */
} ETH_DMARxFrameInfos;

typedef struct
{
	ETH_InitTypeDef Init;          /*!< Ethernet Init Configuration */
	uint32_t LinkStatus;    /*!< Ethernet link status        */
	ETH_DMADescTypeDef *RxDesc;       /*!< Rx descriptor to Get        */
	ETH_DMADescTypeDef *TxDesc;       /*!< Tx descriptor to Set        */
	ETH_DMARxFrameInfos RxFrameInfos;  /*!< last Rx frame infos         */
	volatile HAL_ETH_StateTypeDef State;         /*!< ETH communication state     */
	HAL_LockTypeDef Lock;          /*!< ETH Lock                    */
} ETH_HandleTypeDef;

#define ETH_MAX_PACKET_SIZE    ((uint32_t)1524U)    /*!< ETH_HEADER + ETH_EXTRA + ETH_VLAN_TAG + ETH_MAX_ETH_PAYLOAD + ETH_CRC */
#define ETH_HEADER               ((uint32_t)14U)    /*!< 6 byte Dest addr, 6 byte Src addr, 2 byte length/type */
#define ETH_CRC                   ((uint32_t)4U)    /*!< Ethernet CRC */
#define ETH_EXTRA                 ((uint32_t)2U)    /*!< Extra bytes in some cases */
#define ETH_VLAN_TAG              ((uint32_t)4U)    /*!< optional 802.1q VLAN Tag */
#define ETH_MIN_ETH_PAYLOAD       ((uint32_t)46U)    /*!< Minimum Ethernet payload size */
#define ETH_MAX_ETH_PAYLOAD       ((uint32_t)1500U)    /*!< Maximum Ethernet payload size */
#define ETH_JUMBO_FRAME_PAYLOAD   ((uint32_t)9000U)    /*!< Jumbo frame payload size */

/* 5 Ethernet driver receive buffers are used (in a chained linked list)*/
#ifndef ETH_RXBUFNB
	#define ETH_RXBUFNB             ((uint32_t)5U)     /*  5 Rx buffers of size ETH_RX_BUF_SIZE */
#endif

#ifndef ETH_RX_BUF_SIZE
	#define ETH_RX_BUF_SIZE         ETH_MAX_PACKET_SIZE
#endif

#ifndef ETH_TX_BUF_SIZE
	#define ETH_TX_BUF_SIZE         ETH_MAX_PACKET_SIZE
#endif

/* 5 Ethernet driver transmit buffers are used (in a chained linked list)*/
#ifndef ETH_TXBUFNB
	#define ETH_TXBUFNB             ((uint32_t)5U)      /* 5  Tx buffers of size ETH_TX_BUF_SIZE */
#endif

#define __HAL_LOCK(__HANDLE__)  do { if ((__HANDLE__)->Lock == HAL_LOCKED) { return HAL_BUSY; } else { (__HANDLE__)->Lock = HAL_LOCKED; }} while (0U)

#define __HAL_UNLOCK(__HANDLE__) do{ (__HANDLE__)->Lock = HAL_UNLOCKED; } while (0U)

#define ETH_MAC_ADDRESS0     ((uint32_t)0x00000000U)
#define ETH_MAC_ADDRESS1     ((uint32_t)0x00000008U)
#define ETH_MAC_ADDRESS2     ((uint32_t)0x00000010U)
#define ETH_MAC_ADDRESS3     ((uint32_t)0x00000018U)

/* Ethernet Errors */
#define  ETH_SUCCESS            ((uint32_t)0U)
#define  ETH_ERROR              ((uint32_t)1U)

typedef enum
{
	RESET = 0U,
	SET = !RESET
} FlagStatus, ITStatus;

/**
  * @brief  Bit definition of TDES0 register: DMA Tx descriptor status register
  */
#define ETH_DMATXDESC_OWN                     ((uint32_t)0x80000000U)  /*!< OWN bit: descriptor is owned by DMA engine */
#define ETH_DMATXDESC_IC                      ((uint32_t)0x40000000U)  /*!< Interrupt on Completion */
#define ETH_DMATXDESC_LS                      ((uint32_t)0x20000000U)  /*!< Last Segment */
#define ETH_DMATXDESC_FS                      ((uint32_t)0x10000000U)  /*!< First Segment */
#define ETH_DMATXDESC_DC                      ((uint32_t)0x08000000U)  /*!< Disable CRC */
#define ETH_DMATXDESC_DP                      ((uint32_t)0x04000000U)  /*!< Disable Padding */
#define ETH_DMATXDESC_TTSE                    ((uint32_t)0x02000000U)  /*!< Transmit Time Stamp Enable */
#define ETH_DMATXDESC_CIC                     ((uint32_t)0x00C00000U)  /*!< Checksum Insertion Control: 4 cases */
#define ETH_DMATXDESC_CIC_BYPASS              ((uint32_t)0x00000000U)  /*!< Do Nothing: Checksum Engine is bypassed */
#define ETH_DMATXDESC_CIC_IPV4HEADER          ((uint32_t)0x00400000U)  /*!< IPV4 header Checksum Insertion */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_SEGMENT  ((uint32_t)0x00800000U)  /*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_FULL     ((uint32_t)0x00C00000U)  /*!< TCP/UDP/ICMP Checksum Insertion fully calculated */
#define ETH_DMATXDESC_TER                     ((uint32_t)0x00200000U)  /*!< Transmit End of Ring */
#define ETH_DMATXDESC_TCH                     ((uint32_t)0x00100000U)  /*!< Second Address Chained */
#define ETH_DMATXDESC_TTSS                    ((uint32_t)0x00020000U)  /*!< Tx Time Stamp Status */
#define ETH_DMATXDESC_IHE                     ((uint32_t)0x00010000U)  /*!< IP Header Error */
#define ETH_DMATXDESC_ES                      ((uint32_t)0x00008000U)  /*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define ETH_DMATXDESC_JT                      ((uint32_t)0x00004000U)  /*!< Jabber Timeout */
#define ETH_DMATXDESC_FF                      ((uint32_t)0x00002000U)  /*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define ETH_DMATXDESC_PCE                     ((uint32_t)0x00001000U)  /*!< Payload Checksum Error */
#define ETH_DMATXDESC_LCA                     ((uint32_t)0x00000800U)  /*!< Loss of Carrier: carrier lost during transmission */
#define ETH_DMATXDESC_NC                      ((uint32_t)0x00000400U)  /*!< No Carrier: no carrier signal from the transceiver */
#define ETH_DMATXDESC_LCO                     ((uint32_t)0x00000200U)  /*!< Late Collision: transmission aborted due to collision */
#define ETH_DMATXDESC_EC                      ((uint32_t)0x00000100U)  /*!< Excessive Collision: transmission aborted after 16 collisions */
#define ETH_DMATXDESC_VF                      ((uint32_t)0x00000080U)  /*!< VLAN Frame */
#define ETH_DMATXDESC_CC                      ((uint32_t)0x00000078U)  /*!< Collision Count */
#define ETH_DMATXDESC_ED                      ((uint32_t)0x00000004U)  /*!< Excessive Deferral */
#define ETH_DMATXDESC_UF                      ((uint32_t)0x00000002U)  /*!< Underflow Error: late data arrival from the memory */
#define ETH_DMATXDESC_DB                      ((uint32_t)0x00000001U)  /*!< Deferred Bit */

#define ETH_AUTONEGOTIATION_ENABLE     ((uint32_t)0x00000001U)
#define ETH_AUTONEGOTIATION_DISABLE    ((uint32_t)0x00000000U)

#define ETH_SPEED_10M        ((uint32_t)0x00000000U)
#define ETH_SPEED_100M       ((uint32_t)0x00004000U)

#define ETH_MODE_FULLDUPLEX       ((uint32_t)0x00000800U)
#define ETH_MODE_HALFDUPLEX       ((uint32_t)0x00000000U)

#define ETH_RXPOLLING_MODE      ((uint32_t)0x00000000U)
#define ETH_RXINTERRUPT_MODE    ((uint32_t)0x00000001U)

#define ETH_CHECKSUM_BY_HARDWARE      ((uint32_t)0x00000000U)
#define ETH_CHECKSUM_BY_SOFTWARE      ((uint32_t)0x00000001U)

#define SYSCFG_PMC_MII_RMII_SEL_Pos     (23U)
#define SYSCFG_PMC_MII_RMII_SEL_Msk     (0x1UL << SYSCFG_PMC_MII_RMII_SEL_Pos)  /*!< 0x00800000 */
#define SYSCFG_PMC_MII_RMII_SEL         SYSCFG_PMC_MII_RMII_SEL_Msk            /*!<Ethernet PHY interface selection */

#define ETH_MEDIA_INTERFACE_MII       ((uint32_t)0x00000000U)
#define ETH_MEDIA_INTERFACE_RMII      ((uint32_t)SYSCFG_PMC_MII_RMII_SEL)

#define ETH_WATCHDOG_ENABLE       ((uint32_t)0x00000000U)
#define ETH_WATCHDOG_DISABLE      ((uint32_t)0x00800000U)

#define ETH_JABBER_ENABLE    ((uint32_t)0x00000000U)
#define ETH_JABBER_DISABLE   ((uint32_t)0x00400000U)

#define ETH_DMATXDESC_CHECKSUMBYPASS             ((uint32_t)0x00000000U)   /*!< Checksum engine bypass */
#define ETH_DMATXDESC_CHECKSUMIPV4HEADER         ((uint32_t)0x00400000U)   /*!< IPv4 header checksum insertion  */
#define ETH_DMATXDESC_CHECKSUMTCPUDPICMPSEGMENT  ((uint32_t)0x00800000U)   /*!< TCP/UDP/ICMP checksum insertion. Pseudo header checksum is assumed to be present */
#define ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL     ((uint32_t)0x00C00000U)   /*!< TCP/UDP/ICMP checksum fully in hardware including pseudo header */

#define ETH_DMARXDESC_OWN         ((uint32_t)0x80000000U)  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARXDESC_AFM         ((uint32_t)0x40000000U)  /*!< DA Filter Fail for the rx frame  */
#define ETH_DMARXDESC_FL          ((uint32_t)0x3FFF0000U)  /*!< Receive descriptor frame length  */
#define ETH_DMARXDESC_ES          ((uint32_t)0x00008000U)  /*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE */
#define ETH_DMARXDESC_DE          ((uint32_t)0x00004000U)  /*!< Descriptor error: no more descriptors for receive frame  */
#define ETH_DMARXDESC_SAF         ((uint32_t)0x00002000U)  /*!< SA Filter Fail for the received frame */
#define ETH_DMARXDESC_LE          ((uint32_t)0x00001000U)  /*!< Frame size not matching with length field */
#define ETH_DMARXDESC_OE          ((uint32_t)0x00000800U)  /*!< Overflow Error: Frame was damaged due to buffer overflow */
#define ETH_DMARXDESC_VLAN        ((uint32_t)0x00000400U)  /*!< VLAN Tag: received frame is a VLAN frame */
#define ETH_DMARXDESC_FS          ((uint32_t)0x00000200U)  /*!< First descriptor of the frame  */
#define ETH_DMARXDESC_LS          ((uint32_t)0x00000100U)  /*!< Last descriptor of the frame  */
#define ETH_DMARXDESC_IPV4HCE     ((uint32_t)0x00000080U)  /*!< IPC Checksum Error: Rx Ipv4 header checksum error   */
#define ETH_DMARXDESC_LC          ((uint32_t)0x00000040U)  /*!< Late collision occurred during reception   */
#define ETH_DMARXDESC_FT          ((uint32_t)0x00000020U)  /*!< Frame type - Ethernet, otherwise 802.3    */
#define ETH_DMARXDESC_RWT         ((uint32_t)0x00000010U)  /*!< Receive Watchdog Timeout: watchdog timer expired during reception    */
#define ETH_DMARXDESC_RE          ((uint32_t)0x00000008U)  /*!< Receive error: error reported by MII interface  */
#define ETH_DMARXDESC_DBE         ((uint32_t)0x00000004U)  /*!< Dribble bit error: frame contains non int multiple of 8 bits  */
#define ETH_DMARXDESC_CE          ((uint32_t)0x00000002U)  /*!< CRC error */
#define ETH_DMARXDESC_MAMPCE      ((uint32_t)0x00000001U)  /*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

#define ETH_DMARXDESC_DIC   ((uint32_t)0x80000000U)  /*!< Disable Interrupt on Completion */
#define ETH_DMARXDESC_RBS2  ((uint32_t)0x1FFF0000U)  /*!< Receive Buffer2 Size */
#define ETH_DMARXDESC_RER   ((uint32_t)0x00008000U)  /*!< Receive End of Ring */
#define ETH_DMARXDESC_RCH   ((uint32_t)0x00004000U)  /*!< Second Address Chained */
#define ETH_DMARXDESC_RBS1  ((uint32_t)0x00001FFFU)  /*!< Receive Buffer1 Size */

#define ETH_DMA_IT_TST       ((uint32_t)0x20000000U)  /*!< Time-stamp trigger interrupt (on DMA) */
#define ETH_DMA_IT_PMT       ((uint32_t)0x10000000U)  /*!< PMT interrupt (on DMA) */
#define ETH_DMA_IT_MMC       ((uint32_t)0x08000000U)  /*!< MMC interrupt (on DMA) */
#define ETH_DMA_IT_NIS       ((uint32_t)0x00010000U)  /*!< Normal interrupt summary */
#define ETH_DMA_IT_AIS       ((uint32_t)0x00008000U)  /*!< Abnormal interrupt summary */
#define ETH_DMA_IT_ER        ((uint32_t)0x00004000U)  /*!< Early receive interrupt */
#define ETH_DMA_IT_FBE       ((uint32_t)0x00002000U)  /*!< Fatal bus error interrupt */
#define ETH_DMA_IT_ET        ((uint32_t)0x00000400U)  /*!< Early transmit interrupt */
#define ETH_DMA_IT_RWT       ((uint32_t)0x00000200U)  /*!< Receive watchdog timeout interrupt */
#define ETH_DMA_IT_RPS       ((uint32_t)0x00000100U)  /*!< Receive process stopped interrupt */
#define ETH_DMA_IT_RBU       ((uint32_t)0x00000080U)  /*!< Receive buffer unavailable interrupt */
#define ETH_DMA_IT_R         ((uint32_t)0x00000040U)  /*!< Receive interrupt */
#define ETH_DMA_IT_TU        ((uint32_t)0x00000020U)  /*!< Underflow interrupt */
#define ETH_DMA_IT_RO        ((uint32_t)0x00000010U)  /*!< Overflow interrupt */
#define ETH_DMA_IT_TJT       ((uint32_t)0x00000008U)  /*!< Transmit jabber timeout interrupt */
#define ETH_DMA_IT_TBU       ((uint32_t)0x00000004U)  /*!< Transmit buffer unavailable interrupt */
#define ETH_DMA_IT_TPS       ((uint32_t)0x00000002U)  /*!< Transmit process stopped interrupt */
#define ETH_DMA_IT_T         ((uint32_t)0x00000001U)  /*!< Transmit interrupt */

#define ETH_DMA_FLAG_TST               ((uint32_t)0x20000000U)  /*!< Time-stamp trigger interrupt (on DMA) */
#define ETH_DMA_FLAG_PMT               ((uint32_t)0x10000000U)  /*!< PMT interrupt (on DMA) */
#define ETH_DMA_FLAG_MMC               ((uint32_t)0x08000000U)  /*!< MMC interrupt (on DMA) */
#define ETH_DMA_FLAG_DATATRANSFERERROR ((uint32_t)0x00800000U)  /*!< Error bits 0-Rx DMA, 1-Tx DMA */
#define ETH_DMA_FLAG_READWRITEERROR    ((uint32_t)0x01000000U)  /*!< Error bits 0-write transfer, 1-read transfer */
#define ETH_DMA_FLAG_ACCESSERROR       ((uint32_t)0x02000000U)  /*!< Error bits 0-data buffer, 1-desc. access */
#define ETH_DMA_FLAG_NIS               ((uint32_t)0x00010000U)  /*!< Normal interrupt summary flag */
#define ETH_DMA_FLAG_AIS               ((uint32_t)0x00008000U)  /*!< Abnormal interrupt summary flag */
#define ETH_DMA_FLAG_ER                ((uint32_t)0x00004000U)  /*!< Early receive flag */
#define ETH_DMA_FLAG_FBE               ((uint32_t)0x00002000U)  /*!< Fatal bus error flag */
#define ETH_DMA_FLAG_ET                ((uint32_t)0x00000400U)  /*!< Early transmit flag */
#define ETH_DMA_FLAG_RWT               ((uint32_t)0x00000200U)  /*!< Receive watchdog timeout flag */
#define ETH_DMA_FLAG_RPS               ((uint32_t)0x00000100U)  /*!< Receive process stopped flag */
#define ETH_DMA_FLAG_RBU               ((uint32_t)0x00000080U)  /*!< Receive buffer unavailable flag */
#define ETH_DMA_FLAG_R                 ((uint32_t)0x00000040U)  /*!< Receive flag */
#define ETH_DMA_FLAG_TU                ((uint32_t)0x00000020U)  /*!< Underflow flag */
#define ETH_DMA_FLAG_RO                ((uint32_t)0x00000010U)  /*!< Overflow flag */
#define ETH_DMA_FLAG_TJT               ((uint32_t)0x00000008U)  /*!< Transmit jabber timeout flag */
#define ETH_DMA_FLAG_TBU               ((uint32_t)0x00000004U)  /*!< Transmit buffer unavailable flag */
#define ETH_DMA_FLAG_TPS               ((uint32_t)0x00000002U)  /*!< Transmit process stopped flag */
#define ETH_DMA_FLAG_T                 ((uint32_t)0x00000001U)  /*!< Transmit flag */

#define ETH_DMATXDESC_TBS2  ((uint32_t)0x1FFF0000U)  /*!< Transmit Buffer2 Size */
#define ETH_DMATXDESC_TBS1  ((uint32_t)0x00001FFFU)  /*!< Transmit Buffer1 Size */

#define ETH_DMARXDESC_FRAMELENGTHSHIFT            ((uint32_t)16)
#define ETH_MACCR_CLEAR_MASK    ((uint32_t)0xFF20810FU)
#define ETH_MACFCR_CLEAR_MASK   ((uint32_t)0x0000FF41U)
#define ETH_DMAOMR_CLEAR_MASK   ((uint32_t)0xF8DE3F23U)
#define ETH_MACMIIAR_CR_MASK    ((uint32_t)0xFFFFFFE3U)

#define ETH_INTERFRAMEGAP_96BIT   ((uint32_t)0x00000000U)  /*!< minimum IFG between frames during transmission is 96Bit */
#define ETH_INTERFRAMEGAP_88BIT   ((uint32_t)0x00020000U)  /*!< minimum IFG between frames during transmission is 88Bit */
#define ETH_INTERFRAMEGAP_80BIT   ((uint32_t)0x00040000U)  /*!< minimum IFG between frames during transmission is 80Bit */
#define ETH_INTERFRAMEGAP_72BIT   ((uint32_t)0x00060000U)  /*!< minimum IFG between frames during transmission is 72Bit */
#define ETH_INTERFRAMEGAP_64BIT   ((uint32_t)0x00080000U)  /*!< minimum IFG between frames during transmission is 64Bit */
#define ETH_INTERFRAMEGAP_56BIT   ((uint32_t)0x000A0000U)  /*!< minimum IFG between frames during transmission is 56Bit */
#define ETH_INTERFRAMEGAP_48BIT   ((uint32_t)0x000C0000U)  /*!< minimum IFG between frames during transmission is 48Bit */
#define ETH_INTERFRAMEGAP_40BIT   ((uint32_t)0x000E0000U)  /*!< minimum IFG between frames during transmission is 40Bit */

#define ETH_CARRIERSENCE_ENABLE   ((uint32_t)0x00000000U)
#define ETH_CARRIERSENCE_DISABLE  ((uint32_t)0x00010000U)

#define ETH_RECEIVEOWN_ENABLE     ((uint32_t)0x00000000U)
#define ETH_RECEIVEOWN_DISABLE    ((uint32_t)0x00002000U)

#define ETH_LOOPBACKMODE_ENABLE        ((uint32_t)0x00001000U)
#define ETH_LOOPBACKMODE_DISABLE       ((uint32_t)0x00000000U)

#define ETH_CHECKSUMOFFLAOD_ENABLE     ((uint32_t)0x00000400U)
#define ETH_CHECKSUMOFFLAOD_DISABLE    ((uint32_t)0x00000000U)

#define ETH_RETRYTRANSMISSION_ENABLE   ((uint32_t)0x00000000U)
#define ETH_RETRYTRANSMISSION_DISABLE  ((uint32_t)0x00000200U)

#define ETH_AUTOMATICPADCRCSTRIP_ENABLE     ((uint32_t)0x00000080U)
#define ETH_AUTOMATICPADCRCSTRIP_DISABLE    ((uint32_t)0x00000000U)

#define ETH_BACKOFFLIMIT_10  ((uint32_t)0x00000000U)
#define ETH_BACKOFFLIMIT_8   ((uint32_t)0x00000020U)
#define ETH_BACKOFFLIMIT_4   ((uint32_t)0x00000040U)
#define ETH_BACKOFFLIMIT_1   ((uint32_t)0x00000060U)

#define ETH_DEFFERRALCHECK_ENABLE       ((uint32_t)0x00000010U)
#define ETH_DEFFERRALCHECK_DISABLE      ((uint32_t)0x00000000U)

#define ETH_RECEIVEALL_ENABLE     ((uint32_t)0x80000000U)
#define ETH_RECEIVEAll_DISABLE    ((uint32_t)0x00000000U)

#define ETH_SOURCEADDRFILTER_NORMAL_ENABLE       ((uint32_t)0x00000200U)
#define ETH_SOURCEADDRFILTER_INVERSE_ENABLE      ((uint32_t)0x00000300U)
#define ETH_SOURCEADDRFILTER_DISABLE             ((uint32_t)0x00000000U)

#define ETH_PASSCONTROLFRAMES_BLOCKALL                ((uint32_t)0x00000040U)  /*!< MAC filters all control frames from reaching the application */
#define ETH_PASSCONTROLFRAMES_FORWARDALL              ((uint32_t)0x00000080U)  /*!< MAC forwards all control frames to application even if they fail the Address Filter */
#define ETH_PASSCONTROLFRAMES_FORWARDPASSEDADDRFILTER ((uint32_t)0x000000C0U)  /*!< MAC forwards control frames that pass the Address Filter. */

#define ETH_BROADCASTFRAMESRECEPTION_ENABLE     ((uint32_t)0x00000000U)
#define ETH_BROADCASTFRAMESRECEPTION_DISABLE    ((uint32_t)0x00000020U)

#define ETH_DESTINATIONADDRFILTER_NORMAL    ((uint32_t)0x00000000U)
#define ETH_DESTINATIONADDRFILTER_INVERSE   ((uint32_t)0x00000008U)

#define ETH_PROMISCUOUS_MODE_ENABLE     ((uint32_t)0x00000001U)
#define ETH_PROMISCUOUS_MODE_DISABLE    ((uint32_t)0x00000000U)

#define ETH_MULTICASTFRAMESFILTER_PERFECTHASHTABLE    ((uint32_t)0x00000404U)
#define ETH_MULTICASTFRAMESFILTER_HASHTABLE           ((uint32_t)0x00000004U)
#define ETH_MULTICASTFRAMESFILTER_PERFECT             ((uint32_t)0x00000000U)
#define ETH_MULTICASTFRAMESFILTER_NONE                ((uint32_t)0x00000010U)

#define ETH_UNICASTFRAMESFILTER_PERFECTHASHTABLE ((uint32_t)0x00000402U)
#define ETH_UNICASTFRAMESFILTER_HASHTABLE        ((uint32_t)0x00000002U)
#define ETH_UNICASTFRAMESFILTER_PERFECT          ((uint32_t)0x00000000U)

#define ETH_ZEROQUANTAPAUSE_ENABLE     ((uint32_t)0x00000000U)
#define ETH_ZEROQUANTAPAUSE_DISABLE    ((uint32_t)0x00000080U)

#define ETH_PAUSELOWTHRESHOLD_MINUS4        ((uint32_t)0x00000000U)  /*!< Pause time minus 4 slot times */
#define ETH_PAUSELOWTHRESHOLD_MINUS28       ((uint32_t)0x00000010U)  /*!< Pause time minus 28 slot times */
#define ETH_PAUSELOWTHRESHOLD_MINUS144      ((uint32_t)0x00000020U)  /*!< Pause time minus 144 slot times */
#define ETH_PAUSELOWTHRESHOLD_MINUS256      ((uint32_t)0x00000030U)  /*!< Pause time minus 256 slot times */

#define ETH_UNICASTPAUSEFRAMEDETECT_ENABLE  ((uint32_t)0x00000008U)
#define ETH_UNICASTPAUSEFRAMEDETECT_DISABLE ((uint32_t)0x00000000U)

#define ETH_RECEIVEFLOWCONTROL_ENABLE       ((uint32_t)0x00000004U)
#define ETH_RECEIVEFLOWCONTROL_DISABLE      ((uint32_t)0x00000000U)

#define ETH_TRANSMITFLOWCONTROL_ENABLE      ((uint32_t)0x00000002U)
#define ETH_TRANSMITFLOWCONTROL_DISABLE     ((uint32_t)0x00000000U)

#define ETH_VLANTAGCOMPARISON_12BIT    ((uint32_t)0x00010000U)
#define ETH_VLANTAGCOMPARISON_16BIT    ((uint32_t)0x00000000U)

#define ETH_DROPTCPIPCHECKSUMERRORFRAME_ENABLE   ((uint32_t)0x00000000U)
#define ETH_DROPTCPIPCHECKSUMERRORFRAME_DISABLE  ((uint32_t)0x04000000U)

#define ETH_RECEIVESTOREFORWARD_ENABLE      ((uint32_t)0x02000000U)
#define ETH_RECEIVESTOREFORWARD_DISABLE     ((uint32_t)0x00000000U)

#define ETH_FLUSHRECEIVEDFRAME_ENABLE       ((uint32_t)0x00000000U)
#define ETH_FLUSHRECEIVEDFRAME_DISABLE      ((uint32_t)0x01000000U)

#define ETH_TRANSMITSTOREFORWARD_ENABLE     ((uint32_t)0x00200000U)
#define ETH_TRANSMITSTOREFORWARD_DISABLE    ((uint32_t)0x00000000U)

#define ETH_TRANSMITTHRESHOLDCONTROL_64BYTES     ((uint32_t)0x00000000U)  /*!< threshold level of the MTL Transmit FIFO is 64 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_128BYTES    ((uint32_t)0x00004000U)  /*!< threshold level of the MTL Transmit FIFO is 128 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_192BYTES    ((uint32_t)0x00008000U)  /*!< threshold level of the MTL Transmit FIFO is 192 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_256BYTES    ((uint32_t)0x0000C000U)  /*!< threshold level of the MTL Transmit FIFO is 256 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_40BYTES     ((uint32_t)0x00010000U)  /*!< threshold level of the MTL Transmit FIFO is 40 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_32BYTES     ((uint32_t)0x00014000U)  /*!< threshold level of the MTL Transmit FIFO is 32 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_24BYTES     ((uint32_t)0x00018000U)  /*!< threshold level of the MTL Transmit FIFO is 24 Bytes */
#define ETH_TRANSMITTHRESHOLDCONTROL_16BYTES     ((uint32_t)0x0001C000U)  /*!< threshold level of the MTL Transmit FIFO is 16 Bytes */

#define ETH_FORWARDERRORFRAMES_ENABLE       ((uint32_t)0x00000080U)
#define ETH_FORWARDERRORFRAMES_DISABLE      ((uint32_t)0x00000000U)

#define ETH_FORWARDUNDERSIZEDGOODFRAMES_ENABLE   ((uint32_t)0x00000040U)
#define ETH_FORWARDUNDERSIZEDGOODFRAMES_DISABLE  ((uint32_t)0x00000000U)

#define ETH_RECEIVEDTHRESHOLDCONTROL_64BYTES      ((uint32_t)0x00000000U)  /*!< threshold level of the MTL Receive FIFO is 64 Bytes */
#define ETH_RECEIVEDTHRESHOLDCONTROL_32BYTES      ((uint32_t)0x00000008U)  /*!< threshold level of the MTL Receive FIFO is 32 Bytes */
#define ETH_RECEIVEDTHRESHOLDCONTROL_96BYTES      ((uint32_t)0x00000010U)  /*!< threshold level of the MTL Receive FIFO is 96 Bytes */
#define ETH_RECEIVEDTHRESHOLDCONTROL_128BYTES     ((uint32_t)0x00000018U)  /*!< threshold level of the MTL Receive FIFO is 128 Bytes */

#define ETH_SECONDFRAMEOPERARTE_ENABLE       ((uint32_t)0x00000004U)
#define ETH_SECONDFRAMEOPERARTE_DISABLE      ((uint32_t)0x00000000U)

#define ETH_ADDRESSALIGNEDBEATS_ENABLE      ((uint32_t)0x02000000U)
#define ETH_ADDRESSALIGNEDBEATS_DISABLE     ((uint32_t)0x00000000U)

#define ETH_FIXEDBURST_ENABLE     ((uint32_t)0x00010000U)
#define ETH_FIXEDBURST_DISABLE    ((uint32_t)0x00000000U)

#define ETH_RXDMABURSTLENGTH_1BEAT          ((uint32_t)0x00020000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 1 */
#define ETH_RXDMABURSTLENGTH_2BEAT          ((uint32_t)0x00040000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 2 */
#define ETH_RXDMABURSTLENGTH_4BEAT          ((uint32_t)0x00080000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_RXDMABURSTLENGTH_8BEAT          ((uint32_t)0x00100000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_RXDMABURSTLENGTH_16BEAT         ((uint32_t)0x00200000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_RXDMABURSTLENGTH_32BEAT         ((uint32_t)0x00400000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 32 */
#define ETH_RXDMABURSTLENGTH_4XPBL_4BEAT    ((uint32_t)0x01020000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_RXDMABURSTLENGTH_4XPBL_8BEAT    ((uint32_t)0x01040000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_RXDMABURSTLENGTH_4XPBL_16BEAT   ((uint32_t)0x01080000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_RXDMABURSTLENGTH_4XPBL_32BEAT   ((uint32_t)0x01100000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 32 */
#define ETH_RXDMABURSTLENGTH_4XPBL_64BEAT   ((uint32_t)0x01200000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 64 */
#define ETH_RXDMABURSTLENGTH_4XPBL_128BEAT  ((uint32_t)0x01400000U)  /*!< maximum number of beats to be transferred in one RxDMA transaction is 128 */

#define ETH_TXDMABURSTLENGTH_1BEAT          ((uint32_t)0x00000100U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 1 */
#define ETH_TXDMABURSTLENGTH_2BEAT          ((uint32_t)0x00000200U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 2 */
#define ETH_TXDMABURSTLENGTH_4BEAT          ((uint32_t)0x00000400U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_TXDMABURSTLENGTH_8BEAT          ((uint32_t)0x00000800U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_TXDMABURSTLENGTH_16BEAT         ((uint32_t)0x00001000U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_TXDMABURSTLENGTH_32BEAT         ((uint32_t)0x00002000U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
#define ETH_TXDMABURSTLENGTH_4XPBL_4BEAT    ((uint32_t)0x01000100U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_TXDMABURSTLENGTH_4XPBL_8BEAT    ((uint32_t)0x01000200U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_TXDMABURSTLENGTH_4XPBL_16BEAT   ((uint32_t)0x01000400U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_TXDMABURSTLENGTH_4XPBL_32BEAT   ((uint32_t)0x01000800U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
#define ETH_TXDMABURSTLENGTH_4XPBL_64BEAT   ((uint32_t)0x01001000U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 64 */
#define ETH_TXDMABURSTLENGTH_4XPBL_128BEAT  ((uint32_t)0x01002000U)  /*!< maximum number of beats to be transferred in one TxDMA (or both) transaction is 128 */

#define ETH_DMAENHANCEDDESCRIPTOR_ENABLE              ((uint32_t)0x00000080U)
#define ETH_DMAENHANCEDDESCRIPTOR_DISABLE             ((uint32_t)0x00000000U)

#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_1_1   ((uint32_t)0x00000000U)
#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_2_1   ((uint32_t)0x00004000U)
#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_3_1   ((uint32_t)0x00008000U)
#define ETH_DMAARBITRATION_ROUNDROBIN_RXTX_4_1   ((uint32_t)0x0000C000U)
#define ETH_DMAARBITRATION_RXPRIORTX             ((uint32_t)0x00000002U)

#define ETH_MAX_LOOP	10000

#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif

#ifndef __ALIGN_END
#define __ALIGN_END
#endif

#ifndef __IO
#define __IO volatile
#endif

#define __DSB()	__asm__ volatile("dsb":::"memory");

HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t *RegValue);
HAL_StatusTypeDef HAL_ETH_WritePHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t RegValue);
HAL_StatusTypeDef HAL_ETH_ConfigMAC(ETH_HandleTypeDef *heth, ETH_MACInitTypeDef *macconf);
HAL_StatusTypeDef HAL_ETH_Start( ETH_HandleTypeDef *heth );
HAL_StatusTypeDef HAL_ETH_Stop(ETH_HandleTypeDef *heth);
void HAL_ETH_IRQHandler(ETH_HandleTypeDef *heth);

#endif
