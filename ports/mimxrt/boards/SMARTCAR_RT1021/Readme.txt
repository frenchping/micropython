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
    