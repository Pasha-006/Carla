#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>

#define UDP_PORT_RECEIVE 8811
#define UDP_PORT_SEND 8812
#define BUFFER_SIZE 1024

struct AccInputSigFrame {
    double Trgt_Px_m;
    double Trgt_Vx_mps;
    double Trgt_status;
    double Trgt_Spd_mps[30];
    double Veh_spd_mps;
    double Veh_PathRadius_m;
    double Veh_accel_mps2;
    double SysPwrMd;
    double ACC_ECU_CMD;
};

struct AccOutputSigFrame {
    double a_x_for_trq;
    double Trq_en;
};

double calculate_a_x_for_trq(double Veh_spd_mps, double Trgt_Px_m)
{
    return std::max(0.0,1.0 - (Trgt_Px_m /100.0));
}

double determine_Trq_en(double Trgt_status)
{
    if(Trgt_status == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int main()
{
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];

    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std :: cerr << "Failed to create socket" << std::endl;
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_PORT_RECEIVE);

    if(bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sockfd);
        return -1;
    }

    while(true)
    {
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &addr_len);
        if(n < 0)
        {
            std::cerr << "Failed to receive data" << std::endl;
            continue;
        }
        else
        {
            std::cout << "Received data" << std::endl;
        }

        AccInputSigFrame inputData;
        memcpy(&inputData, buffer, sizeof(AccInputSigFrame));

        AccOutputSigFrame outputData;
        outputData.a_x_for_trq = calculate_a_x_for_trq(inputData.Veh_spd_mps,inputData.Trgt_Px_m);
        outputData.Trq_en = determine_Trq_en(inputData.Trgt_status);

        sendto(sockfd, (const char *)&outputData, sizeof(outputData), 0, (const struct sockaddr *)&client_addr, addr_len);
    }

    close(sockfd);
    return 0;
}


