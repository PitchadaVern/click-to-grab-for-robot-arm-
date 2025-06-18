/*-------------C++-----------------*/
#include <iostream>
#include <winsock2.h>
#include <iostream>
#include <fstream>
#include <conio.h> 

#include <ntx.h>
#include <ntxbrow.h>

#define NAME_TRAJECTORY  "ProcContRT"
#define NAME_MEMCONT_SENSOR "Mem_SENSOR"
#define NAME_SEMMEM_SENSOR  "Sem_SENSOR"


typedef struct _SHAREDMEM_SENSOR
{
	double points[4];
} SHAREDMEM_SENSOR;

SHAREDMEM_SENSOR* pSharedMem_SENSOR;

int scnt = 0;
int CNTMAX = 300; //セマフォ取得が300ループできなければ終了

NTXHANDLE hNtx;
NTXHANDLE hRootProcess;
NTXHANDLE hTrj;
NTXHANDLE hMem_SENSOR;
NTXHANDLE hSem_SENSOR;
void *pMem_SENSOR;

#pragma comment(lib, "ws2_32.lib")

#define PORT 12345

int main() {
	WSADATA wsaData;
	SOCKET serverSocket, clientSocket;
	struct sockaddr_in serverAddr, clientAddr;
	int addrLen = sizeof(clientAddr);
	char buffer[32];

	//t sama

	int robot_state = 0;
	int update_robot_state = 0;

	double start_signal = 0;


	// INtime側共有メモリへのアクセス準備
	bool shared_memory_ok = false;

	//共有メモリが取得できるまで何回も実行する
	while (!shared_memory_ok) {
		//get node
		hNtx = ntxGetLocationByName("NodeA");
		if (ntxGetRtStatus(hNtx) != E_OK)
		{
			std::cout << "Cannot find NodeA!\n";
			Sleep(100);
			//return -1;
			continue;
		}

		//get root process
		if ((hRootProcess = ntxGetRootRtProcess(hNtx)) == NTX_BAD_NTXHANDLE)
		{
			std::cout << "Cannot get RootRtProcess!\n";
			Sleep(100);
			//return -1;
			continue;
		}

		//look-up handle of shared memory
		while ((hTrj = ntxLookupNtxhandle(hRootProcess, NAME_TRAJECTORY, 1)) == NTX_BAD_NTXHANDLE);

		//look-up shared memory
		if ((hMem_SENSOR = ntxLookupNtxHandle(hTrj, NAME_MEMCONT_SENSOR, WAIT_FOREVER)) == NTX_BAD_NTXHANDLE)
		{
			std::cout << "Cannot look-up handle of shared memory!\n";
			Sleep(100);
			//return -1;
			continue;
		}

		//shared memory mapping
		if ((pMem_SENSOR = ntxMapRtSharedMemory(hMem_SENSOR)) == NULL)
		{
			std::cout << "Cannot map rtshared memory!\n";
			Sleep(100);
			//return -1;
			continue;
		}

		pSharedMem_SENSOR = (SHAREDMEM_SENSOR*)pMem_SENSOR;


		//look-up semaphore for shared memory exclusion control
		if ((hSem_SENSOR = ntxLookupNtxhandle(hTrj, NAME_SEMMEM_SENSOR, WAIT_FOREVER)) == NTX_BAD_NTXHANDLE)
		{
			std::cout << "Cannot look-up semaphore for shared memory exclusion control!\n";
			Sleep(100);
			//return -1;
			continue;
		}
		shared_memory_ok = true;
	}

	/*-------共有メモリとの通信処理--------*/
	while (1) {

		//get semaphore for shared memory
		if (ntxWaitForRtSemaphore(hSem_SENSOR, 1, WAIT_FOREVER) == NTX_ERROR)
		{
			if (++scnt > CNTMAX)
			{
				std::cout << "Semaphore error!\n";
				Sleep(1000);
				break;
			}
		}
		else
		{
			scnt = 0; //counter reset
					  // Winsock縺ｮ蛻晄悄蛹?
			if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
				std::cerr << "WSAStartup failed.\n";
				return 1;
			}

			// 繧ｽ繧ｱ繝?繝井ｽ懈??
			serverSocket = socket(AF_INET, SOCK_STREAM, 0);
			if (serverSocket == INVALID_SOCKET) {
				std::cerr << "Socket creation failed.\n";
				WSACleanup();
				return 1;
			}

			// 繧｢繝峨Ξ繧ｹ險ｭ螳?
			serverAddr.sin_family = AF_INET;
			serverAddr.sin_addr.s_addr = INADDR_ANY;
			serverAddr.sin_port = htons(PORT);

			// 繝舌う繝ｳ繝?
			if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
				std::cerr << "Bind failed.\n";
				closesocket(serverSocket);
				WSACleanup();
				return 1;
			}

			// 蠕?縺｡蜿励￠
			if (listen(serverSocket, 3) == SOCKET_ERROR) {
				std::cerr << "Listen failed.\n";
				closesocket(serverSocket);
				WSACleanup();
				return 1;
			}

			std::cout << "Waiting for Python client on port " << PORT << "...\n";

			// 謗･邯壼女縺台ｻ倥￠
			clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &addrLen);
			if (clientSocket == INVALID_SOCKET) {
				std::cerr << "Accept failed.\n";
				closesocket(serverSocket);
				WSACleanup();
				return 1;
			}

			//新しく書き換えた
			while (true) {
				int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
				if (bytesReceived == sizeof(buffer)) {
					// バイナリデータをdouble配列に変換
					memcpy(pSharedMem_SENSOR->points, buffer, sizeof(buffer));

					// デバッグ用出力
					std::cout << "Received: ";
					for (int i = 0; i < 4; i++) {
						std::cout << pSharedMem_SENSOR->points[i] << " ";
					}
					std::cout << std::endl;
				}
				else if (bytesReceived <= 0) {
					std::cerr << "Connection closed.\n";
					break;
				}
			}

			// 繧ｯ繝ｭ繝ｼ繧ｺ
			closesocket(clientSocket);
			closesocket(serverSocket);
			WSACleanup();
			//release semaphore for shared memory
			if (ntxReleaseRtSemaphore(hSem_SENSOR, 1) != E_OK)
			{
				ntxReleaseRtSemaphore(hSem_SENSOR, 0);
			}
		}
	}
	Sleep(1000);

	/*---------------------通信処理の終了--------------------*/


	return 0;
}