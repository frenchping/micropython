For Seekfree Smartcard board, i.MX RT1021-144pin.

UART pin assignment:
    RX/TX      HW-UART    Logical UART  RT1021 pin          Board Pin
    DEBUG USB  LPUART1 -> 0             AD_B0_06/AD_B0_07   B6/B7       # On core-board
    C22/C23    LPUART2 -> 1             EMC_22/EMC_23       C22/C23     # Do not use
    C6/C7      LPUART3 -> 2             EMC_06/EMC_07       C6/C7       # Camera
    B10/B11    LPUART5 -> 3             AD_B0_10/AD_B0_11   B10/B11     # Do not user (Display)
    D22/D23    LPUART8 -> 4             SD_B1_02/SD_B1_03   D22/D23     # Not on base-board
    B26/B27    LPUART4 -> 5             AD_B1_10/AD_B1_11   B26/B27     # Do not use (Camera)
    D17/D18    LPUART7 -> 6             SD_B0_04/SD_B0_05   D17/D18     # Wireless UART


ADC pins:
            ADC1    ADC2
    B12       0             # LCD-MOSI
    B13               0     # LCD-BL
    B14       1       1     # Battery
    B15       2       2     # NC

    B17       3             # AMP
    B18               3     # AMP
    B19       4             # AMP
    B20               4     # AMP
    B21       5       5     # Camera VSY
    B22       6       6     # Camera HREF
    B23       7       7     # Camera PCLK
    B24       8       8     # Camera D7
    B25       9       9     # Camera D6
    B26      10      10     # Camera D5
    B27      11      11     # Camera D4
    B28      12      12     # Camera D3
    B29      13      13     # Camera D2
    B30      14      14     # Camera D1
    B31      15      15     # Camera D0