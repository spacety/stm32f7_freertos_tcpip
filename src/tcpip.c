#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <time.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rng.h>

#include <libopencm3/ethernet/mac.h>
#include <libopencm3/ethernet/phy_lan87xx.h>

#include "tcpip.h"
#include "miniprintf.h"

char *usart_rx_buf = NULL;
int usart_rx_len = 0, escape_mode = 0;
gosh_cmd_handler gosh_cb = NULL;
BaseType_t woken = pdTRUE;
char gosh_prompt[64] = "# ";
QueueHandle_t g_usart_rx_queue = NULL;
uint32_t usart_console = 0;
uint8_t freertos_started = 0;
TaskHandle_t usart_handle = 0, usart_cmd_handle = 0;
uint8_t usart_console_enabled = 0;

#define USART_RX_MAX		16

struct usart_data {
	uint32_t usart;
	uint8_t c[USART_RX_MAX];
	int len;
};

/* 00:1e:37:d0:53:55 */
const uint8_t ucMACAddress[6] = {0x00, 0x1e, 0x37, 0xd0, 0x53, 0x55};
const uint8_t ucIPAddress[4] = {192, 168, 16, 200};
const uint8_t ucNetMask[4] = {255, 255, 255, 0};
const uint8_t ucGatewayAddress[4] = {192, 168, 16, 1};
const uint8_t ucDNSServerAddress[4] = {114, 114, 114, 114};

extern const struct rcc_clock_scale rcc_3v3[RCC_CLOCK_3V3_END];
static void
clock_setup(void)
{
	rcc_clock_setup_hsi(&(rcc_3v3[0]));

	rcc_osc_on(RCC_LSI);
}

int
std_printf(const char *format, ...)
{
	va_list args;
	int rc;

	if (usart_console == 0 || usart_console_enabled != MAGIC_55)
		return 0;

	va_start(args, format);
	rc = mini_vprintf_cooked(console_putc, format, args);
	va_end(args);
	return rc;
}

#define USART_CR1_FIFOEN		BIT29

#define USART_CR3_RXFTIE_EN		BIT28
#define USART_CR3_RXFIFO_SHIFT		25
#define USART_FIFO_THRESH_RX_FULL	5 /* 8/8 */
#define USART_FIFO_THRESH_SEVENEIGTH	4 /* 7/8 */
#define USART_FIFO_THRESH_THREEQTR	3 /* 3/4 */
#define USART_FIFO_THRESH_HALF		2 /* 1/2 */
#define USART_FIFO_THRESH_QUARTER	1 /* 1/4 */
#define USART_FIFO_THRESH_EIGTH		0 /* 1/8 */

void
setup_usart_speed(uint32_t usart, uint32_t baudrate, int enable_rxfifo)
{
	if ((usart != USART1) && (usart != USART2) && (usart != USART3))
		return;

	if (baudrate == 0)
		baudrate = 115200;

	usart_set_baudrate(usart, baudrate);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	/* don't enable usart tx/rx here to drop bogus data in tx/rx line */
	/* usart_set_mode(usart, USART_MODE_TX_RX); */
	usart_set_parity(usart, USART_PARITY_NONE);

	if (enable_rxfifo) {
		USART_CR1(usart) |= USART_CR1_FIFOEN; /* enable FIFO */
		USART_CR3(usart) |= (USART_FIFO_THRESH_SEVENEIGTH << USART_CR3_RXFIFO_SHIFT); /* RX FIFO 7/8 */
		USART_CR3(usart) |= USART_CR3_RXFTIE_EN; /* enable rx fifo interrupt */
		USART_CR1(usart) |= USART_CR1_IDLEIE; /* enable idle interrupt */
	}

	/* Enable USART Receive interrupt. */
	usart_enable_rx_interrupt(usart);
}

void
process_cmd(char *cmd)
{
	int len, argc, pos;
	char *argv[MAX_ARGS], *p;

	if (cmd == NULL)
		return;
	len = strlen(cmd);
	argc = pos = 0;
	/* to find count of arguments */
	while (pos < len && argc < MAX_ARGS) {
		/* strip prefix ' ' & '\t' */
		while (pos < len && ((cmd[pos] == ' ') || (cmd[pos] == '\t')))
			pos ++;

		if (pos == len || cmd[pos] == '\0')
			break;
		p = cmd + pos;
		argv[argc ++] = p;

		while (pos < len && ((cmd[pos] != ' ') && (cmd[pos] != '\t')))
			pos ++;

		if (pos == len) break;
		else cmd[pos ++] = '\0';
	}

	if (argc > 0 && gosh_cb != NULL)
		gosh_cb(argv, argc);
	console_puts(gosh_prompt); /* cmd line prefix */
}

void
handle_console_input(char data)
{
	int finish = 0, i, r = 1;

	if (usart_rx_buf == NULL) /* buffer is not ready */
		return;

	if (data == 0x0) {
		/* input NULL character */
		usart_rx_len = 0;
		return;
	}

	if (data == '\r' || data == '\n') {
		/* end of line */
		usart_rx_buf[usart_rx_len] = '\0';
		finish = 1;
	} else if ((data == 0x08) || (data == 0x7f)) {
		/* backspace */
		if (usart_rx_len > 0)
			usart_rx_len --;
	} else if (data == 0x17) { /* ^W Erase Word */
		if (usart_rx_len > 0) {
			for (i = usart_rx_len - 1; i >= 0; i --) {
				if (usart_rx_buf[i] == ' ')
					break;
			}
			if (i > 0)
				usart_rx_len = i + 1;
			else
				usart_rx_len = 0;
		}
	} else if (data == 0x15) { /* ^U Erase Line */
		usart_rx_len = 0;
	} else {
		usart_rx_buf[usart_rx_len ++] = data;
	}

	if (r == 1)
		escape_mode = 0;

	usart_rx_buf[usart_rx_len] = '\0'; /* terminate cmd line */
	if (finish == 1 || (usart_rx_len >= (MAX_USART_RX_LEN - 1))) {
		if (usart_rx_len > 0) {
		}
		console_puts("\r\n"); /* new line */
		process_cmd(usart_rx_buf);
		usart_rx_len = 0;
	} else {
		console_puts("\r"); /* clear current input buffer */
		console_puts(gosh_prompt);
		console_puts(usart_rx_buf);
	}
}

void
setup_gosh_callback(gosh_cmd_handler f)
{
	gosh_cb = f;
}

void
generic_usart_handler(void *args)
{
	int r = 0, len, i;
	struct usart_data d;

	while (1) {
		r = xQueueReceive(g_usart_rx_queue, &d, portMAX_DELAY);
		if (r != pdPASS)
			continue;
		len = uxQueueMessagesWaiting(g_usart_rx_queue);
		do {
			for (i = 0; i < d.len; i ++)
				handle_console_input(d.c[i]);

			if (len > 0) {
				r = xQueueReceive(g_usart_rx_queue, &d, 0);
				len --;
			} else {
				r = pdFAIL;
			}
		} while (r == pdPASS);
	}
}

void
start_usart_task(void)
{
	if (usart_console != 0) {
		g_usart_rx_queue = xQueueCreate(512, sizeof(struct usart_data));
		usart_rx_buf = pvPortMalloc(MAX_USART_RX_LEN + 4);
		memset(usart_rx_buf, 0, MAX_USART_RX_LEN + 4);
	}

	if ((usart_console != 0) && (pdPASS != xTaskCreate(generic_usart_handler, "UART", 800, NULL, 2 | portPRIVILEGE_BIT, &usart_handle))) {
		std_printf("GOSH TASK ERROR\n");
	}

}

void
generic_usart_isr_handler(uint32_t usart)
{
	struct usart_data d;

	if (usart == 0)
		return;

	d.usart = usart;
	d.len = 0;

	if ((USART_ISR(usart) & USART_ISR_IDLE) != 0) {
		/* CLEAR IDLEIE FLAG */
		USART_ICR(usart) |= USART_ICR_IDLECF;
	}

	while ((d.len < USART_RX_MAX) && (USART_ISR(usart) & USART_ISR_RXNE) != 0) {
		d.c[d.len ++] = usart_recv(usart);
	}

	if ((d.len > 0) && g_usart_rx_queue && (uxQueueSpacesAvailable(g_usart_rx_queue) > 0)) {
		xQueueSendToBackFromISR(g_usart_rx_queue, &d, &woken);
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(usart) & USART_CR1_TXEIE) != 0) &&
		((USART_ISR(usart) & USART_ISR_TXE) != 0)) {
		/* Indicate that we are sending out data. */
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(usart) &= ~USART_CR1_TXEIE;
	}
}

void
usart3_isr(void)
{
	generic_usart_isr_handler(USART3);
}

int
usart_timeout_putc(uint32_t usart, char c)
{
	int i = 0;

	if (usart == 0)
		return 0;

	/* uses blocking send for kiss protocol */
	while ((i < USART_LOOP) && ((USART_ISR(usart) & USART_ISR_TXE) == 0))
		i ++;
	if (i < USART_LOOP) {
		usart_send(usart, c);
		return 0;
	}
	return 1;
}

void
usart_send_hex(uint32_t usart, uint8_t *ptr, int len, int mode, int skip_leading_zero)
{
	uint8_t c, d;
	int i;
	int first = 1;

	if (ptr == NULL || len <= 0 || usart == 0)
		return;

	for (i = 0; i < len; i ++) {
		c = ptr[i];
		if (skip_leading_zero == 1 && first == 1 && c == 0)
			continue;
		first = 0;
		if (mode == 1) { /* hex mode */
			d = (c >> 4) & 0x0f;
			if (d >= 0xa)
				d += 'A' - 0xa;
			else
				d += '0';
			usart_timeout_putc(usart, d);

			d = c & 0x0f;
			if (d >= 0xa)
				d += 'A' - 0xa;
			else
				d += '0';
			usart_timeout_putc(usart, d);
		} else { /* text mode */
			usart_timeout_putc(usart, c);
		}
	}
}

void
console_puts(char *ptr)
{
	if (ptr == NULL || usart_console == 0)
		return;

	while (ptr[0]) {
		usart_timeout_putc(usart_console, ptr[0]);
		ptr ++;
	}
}

void
usart_send_string(uint32_t usart, uint8_t *ptr)
{
	if (ptr == NULL || usart == 0)
		return;

	while (ptr[0]) {
		usart_timeout_putc(usart, ptr[0]);
		ptr ++;
	}
}

void
console_putc(char c)
{
	usart_timeout_putc(usart_console, c);
}

/**
 * Dumps a chunk of memory to the screen
 */
void
hex_dump(char *src, int len)
{
	int i, j, k;
	char text[17];

	text[16] = '\0';
	std_printf("%p : ", src);
	for (i = 0, j = 0; i < len; i ++) {
		j ++;
		std_printf("%02X ", ((volatile unsigned char *)src)[i]);
		if (j == 8)
			std_printf(" ");
		if (j == 16 || (i == (len - 1))) {
			if (j < 8)
				std_printf(" ");
			for (k = j; k < 16; k ++) {
				std_printf("   ");
			}

			memcpy(text, &((char *)src)[i + 1 - j], j);
			for (k = 0; k < j; k ++) {
				if ((text[k] < 32) || (text[k] > 126)) {
					text[k] = '.';
				}
			}
			text[j] = '\0';
			if (j == 16)
				std_printf(" |%s|\n", text);
			else
				std_printf(" |%s\n", text);
			if (i < (len - 1)) {
				std_printf("%p : ", src + i + 1);
			}
			j = 0;
		}
	}
	if (i % 16)
		std_printf("\n");
}

extern uint32_t device_flash_size;
extern char *flash_memory_ptr;

static void
simple_fault_handler(const char s)
{
	std_printf("\n[%c]:\nHFSR = 0x%x, CFSR = 0x%x, MMFAR = 0x%x, BFAR = 0x%x, AFSR = 0x%x, DFSR = %x, SCB_SHCSR = 0x%x, FLASH_OPTCR = 0x%x\n",
			s, SCB_HFSR, SCB_CFSR, SCB_MMFAR, SCB_BFAR, SCB_AFSR,
			(*((volatile unsigned long *)(0xE000ED30))), SCB_SHCSR, FLASH_OPTCR);
	while (1);
}

void
usage_fault_handler(void)
{
	simple_fault_handler('U');
}

void
hard_fault_handler(void)
{
	simple_fault_handler('H');
}

void
mem_manage_handler(void)
{
	simple_fault_handler('M');
}

extern void vPortSVCHandler( void ) __attribute__ (( naked ));
extern void xPortPendSVHandler( void ) __attribute__ (( naked ));
extern void xPortSysTickHandler( void );

void sv_call_handler(void) {
	vPortSVCHandler();
}

void pend_sv_handler(void) {
	xPortPendSVHandler();
}

void sys_tick_handler(void) {
	xPortSysTickHandler();
}

void delay_us(uint32_t us)
{
	for (int i = 0; i < us; i++)
		__asm__("nop");
}

void
setup_usart3(void)
{
	/* USART3, PD8, PD9 af7 */
	rcc_periph_clock_enable(RCC_GPIOD);

	rcc_periph_clock_enable(RCC_USART3);

	/* Setup GPIO pins for USART3 transmit. */
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);

	/* Setup USART3 RX/TX pin as alternate function. */
	gpio_set_af(GPIOD, GPIO_AF7, GPIO9 | GPIO8);

	/* can't has high priority than MAX_SYSCALL_INTERRUPT_PRIORITY */
	nvic_set_priority(NVIC_USART3_IRQ, IRQ2NVIC_PRIOR(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY));
	nvic_enable_irq(NVIC_USART3_IRQ);

	setup_usart_speed(USART3, 115200, 0);
}
static void
setup_gosh_prompt(void)
{
	char tmp[16];

	if (FreeRTOS_IsNetworkUp() == pdTRUE) {
		FreeRTOS_inet_ntoa(FreeRTOS_GetIPAddress(), tmp);
		snprintf(gosh_prompt, 63, "[UP | %s] # ", tmp);
	} else {
		strcpy(gosh_prompt, "[DOWN] # ");
	}
}

void
process_tcpip_cmd(char **argv, int argc)
{
	if (argv == NULL || argc <= 0)
		return;

	if (strcasecmp(argv[0], "help") == 0) {
		std_printf("cmd: help reset rng\n");
	} else if (strcasecmp(argv[0], "reset") == 0) {
		scb_reset_system();
	} else if (strcasecmp(argv[0], "rng") == 0) {
		uint32_t r;
		if (rng_get_random(&r)) {
			std_printf("NEW RNG NUMBER 0x%x\n", r);
		} else {
			std_printf("FAIL TO GET RNG NUMBER\n");
		}
	} else {
		std_printf("unknown cmd: %s, argc = %d\n", argv[0], argc);
	}

	setup_gosh_prompt();
}

static void
init_task(void *unused)
{
	setup_gosh_prompt();

	start_usart_task();

	vTaskDelay(pdMS_TO_TICKS(100));

	console_puts(gosh_prompt); /* cmd line prefix */

	freertos_started = MAGIC_55; /* set freertos started flag to 0x55 */

	vTaskDelete(NULL);
}

void
init_usart(int usart)
{
	if (usart == USART3)
		setup_usart3();
}

void
setup_ethernet(void)
{
	/* PG11 - RMII_TX_EN
	 * PG13 - RMII_TXD0
	 * PB13 - RMII_TXD1
	 *
	 * PC4 - RMII_RXD0
	 * PC5 - RMII_RXD1
	 *
	 * PA7 - RMII_CRS_DV
	 *
	 * PC1 - RMII_MDC
	 * PA2 - RMII_MDIO
	 * PA1 - RMII_REF_CLK
	 */

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOG);

	rcc_periph_clock_enable(RCC_SYSCFG);
	/* SELECT RMII */
	SYSCFG_PMC |= AFIO_MAPR_MII_RMII_SEL;

	rcc_periph_reset_pulse(RCC_ETHMAC);
	rcc_periph_clock_enable(RCC_ETHMAC);
	rcc_periph_clock_enable(RCC_ETHMACTX);
	rcc_periph_clock_enable(RCC_ETHMACRX);

	/* PA1/PA2/PA7 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF11, GPIO1 | GPIO2 | GPIO7);

	/* PB13 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13);
	gpio_set_af(GPIOB, GPIO_AF11, GPIO13);

	/* PC1/PC4/PC5 */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1 | GPIO4 | GPIO5);
	gpio_set_af(GPIOC, GPIO_AF11, GPIO1 | GPIO4 | GPIO5);

	/* PG11/PG13 */
	gpio_mode_setup(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO13);
	gpio_set_af(GPIOG, GPIO_AF11, GPIO11 | GPIO13);

	nvic_set_priority(NVIC_ETH_IRQ, IRQ2NVIC_PRIOR(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY));
	nvic_enable_irq(NVIC_ETH_IRQ);
}

void
disable_ethernet(void)
{
	rcc_periph_reset_pulse(RCC_ETHMAC);
}

void
setup_rng(void)
{
	RCC_DCKCFGR2 &= ~RCC_DCKCFGR2_CK48MSEL; /* SELECT PLL AS 48M CLOCK */
	rcc_periph_clock_enable(RCC_RNG);
	rng_interrupt_disable();
	rng_enable(); /* ENABLE RNG NOW */
}

int
main(void)
{
	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

	/* setup global clock first */
	clock_setup();

	/* USART3 */
	usart_console = USART3;
	init_usart(usart_console);
	usart_enable(usart_console);
	usart_set_mode(usart_console, USART_MODE_TX_RX);
	usart_console_enabled = MAGIC_55;

	setup_ethernet();
	setup_rng();

	if (pdPASS != xTaskCreate(init_task, "INIT", 500, NULL, 4 | portPRIVILEGE_BIT, NULL)) {
		std_printf("init task error\n");
	}

	setup_gosh_callback(process_tcpip_cmd);
	FreeRTOS_IPInit(ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress);

	/* START SCHEDULER */
	vTaskStartScheduler();
}

void
vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
	static BaseType_t xTasksAlreadyCreated = pdFALSE;

	/* Both eNetworkUp and eNetworkDown events can be processed here. */
	if (eNetworkEvent == eNetworkUp) {
		/* Create the tasks that use the TCP/IP stack if they have not already been created. */
		if ( xTasksAlreadyCreated == pdFALSE ) {
			/* For convenience, tasks that use FreeRTOS+TCP can be created here
			 * to ensure they are not created before the network is usable.
			 */
			xTasksAlreadyCreated = pdTRUE;
		}
	}
}

void vLoggingPrintf(const char *fmt, ... )
{
	std_printf(fmt);
}

BaseType_t
xApplicationGetRandomNumber(uint32_t *r)
{
	if (rng_get_random(r))
		return pdTRUE;
	else
		return pdFALSE;
}

uint32_t
ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress, uint16_t usSourcePort, uint32_t ulDestinationAddress, uint16_t usDestinationPort)
{
	uint32_t r = 0;
	if (rng_get_random(&r))
		return r;
	else
		return 0;
}

void
vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier )
{
}
