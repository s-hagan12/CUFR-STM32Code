# CUFR-STM32Code 2025
Project Files for Live Telemetry Testing

Each Folder has a different Project.

SD_CARD_Integration contains the code for reading and writing to the MicroSD Card.
- The Folder of FATFS Target Files is the most important part and contains a working driver
- Have successfully read a txt file, and written a txt and csv file
SD_CARD_MANUAL contains a working initialisation sequence for SPI
- This code is not at all clean and was designed to be functional and test whether we can initialise the SD card. Not generalisable
UART_To_Radio successfully transmit data to the other radio. (Does not read)
