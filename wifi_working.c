#include <libpynq.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

int send_data(int x, int y, char* type, int size, char* colour){
    /*********** Read input from terminal **************/
        printf(""); //PUT HERE THE DATA WITH VARIABLES
        fflush(NULL); //Flush the terminal buffer
        char string[?length];

        sprintf(string, "x: %d, y: %d, type: %s, size: %d, colour: %s", x, y, type, size, colour);
        
        for(uint32_t i = 0; i < length; i++)
        {
            uart_send(UART0, string[i]); //Send the payload bytes
        }    
    /***************************************************/
}

int main()
{
    switchbox_init();
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    // initialize UART 0
    uart_init(UART0);
    // flush FIFOs of UART 0
    uart_reset_fifos(UART0);
    
    while (1)
    {           
        /*********** Write output to terminal **************/
        if(uart_has_data(UART0))
            {
            uint32_t i = 0;    
            char* buffer = (char*) malloc(sizeof(char) * length);
            while(i < length)
            {
                buffer[i] = (char)uart_recv(UART0); // Read the payload into memory
                i++;
            }
            //printf("  >%s\n", buffer); DEBUG

            if(strcmp(buffer, "yes") == 0){
                send_data(x, y, type, size, colour);
            }
            
            fflush(NULL); //Flush the terminal buffer
            free(buffer);
        }
        /***************************************************/
    }
    fflush(NULL);
    uart_reset_fifos(UART0);
    uart_destroy(UART0);
    // clean up after use
    pynq_destroy();
    return EXIT_SUCCESS;
}
