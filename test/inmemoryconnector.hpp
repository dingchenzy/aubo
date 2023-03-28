#pragma once
#include <jsonrpccxx/iclientconnector.hpp>
#include <jsonrpccxx/server.hpp>
#include <iostream>

// the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET       "\033[0m"
#define BLACK       "\033[30m"        /* Black */
#define RED         "\033[31m"        /* Red */
#define GREEN       "\033[32m"        /* Green */
#define YELLOW      "\033[33m"        /* Yellow */
#define BLUE        "\033[34m"        /* Blue */
#define MAGENTA     "\033[35m"        /* Magenta */
#define CYAN        "\033[36m"        /* Cyan */
#define WHITE       "\033[37m"        /* White */
#define BOLDBLACK   "\033[1m\033[30m" /* Bold Black */
#define BOLDRED     "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m" /* Bold White */

// This class is server and client connector at the same time.
class InMemoryConnector : public jsonrpccxx::IClientConnector
{
public:
    explicit InMemoryConnector(jsonrpccxx::JsonRpcServer &server)
        : server(server)
    {
    }
    std::string Send(const std::string &request) override
    {
        auto str = server.HandleRequest(request);

        std::cout << YELLOW << "---> " << request << std::endl;
        std::cout << BLUE << "<--- " << str << RESET << std::endl;
        return str;
    }

private:
    jsonrpccxx::JsonRpcServer &server;
};
