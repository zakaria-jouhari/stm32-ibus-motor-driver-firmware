/*
 * net_config.h  -  Static IP scheme for the Pi <-> STM32 Ethernet/UDP link.
 *
 * Replaced the UART7 cmd_vel/odometry link once cable routing across the
 * full chassis made loose jumper wires impractical -- see cmd.h/odom.h for
 * the datagram formats. Static IPs, not DHCP: this is a direct point-to-point
 * link, there's no DHCP server to depend on.
 *
 * These constants must match the Pi side (agribot_base's stm32_ip/cmd_port/
 * odom_port launch parameters) -- change here AND there together.
 */
#ifndef NET_CONFIG_H
#define NET_CONFIG_H

/* This board's own static IP: 192.168.10.2 */
#define NET_STM32_IP0 192
#define NET_STM32_IP1 168
#define NET_STM32_IP2 10
#define NET_STM32_IP3 2

#define NET_NETMASK0 255
#define NET_NETMASK1 255
#define NET_NETMASK2 255
#define NET_NETMASK3 0

/* Gateway isn't really used on a direct 2-device link; set to the Pi's
 * address for completeness. */
#define NET_GW0 192
#define NET_GW1 168
#define NET_GW2 10
#define NET_GW3 1

/* The Pi's static IP: 192.168.10.1 */
#define NET_PI_IP0 192
#define NET_PI_IP1 168
#define NET_PI_IP2 10
#define NET_PI_IP3 1

#define NET_CMD_PORT   5005u   /* Pi -> STM32, cmd_vel (this board listens here)  */
#define NET_ODOM_PORT  5006u   /* STM32 -> Pi, odometry (Pi listens here)          */

#endif /* NET_CONFIG_H */
