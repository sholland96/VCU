/* odroid_shutdown.h — tells the Odroid to shut down gracefully before RELAY_ODROID_PIN cuts
 * its power. Separate TCP port from realdash_tcp.cpp's RealDash feed (port 35000, single
 * client, a new connection replaces the old one) so this signal can't be kicked off by, or
 * kick off, the RealDash connection.
 */
#pragma once
#include <stdint.h>

void odroidShutdownInit();    // call once from setup(), after realdashInit() (same Ethernet.begin())
void odroidShutdownService(); // call from loop() — accepts the Odroid-side watcher's connection
void odroidShutdownSignal();  // call once when KL15R goes low — writes the shutdown byte
