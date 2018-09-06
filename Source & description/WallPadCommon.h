/////////////////////////////////////////////////////////////////////////////////////
#define  nMAX_NAME   40
#define nMAX_INIT_CHK 40
/////////////////////////////////////////////////////////////////////////////////////


typedef enum typedefMODE
{
  eMODE_NONE  = 0,  
  eMODE_SEARCH_AP,  
  eMODE_AP,     // AP MODE
  eMODE_CONNECT,    // Normal 
  eMODE_NORMAL    // Normal 
} eMODE;

/**
 * @brief WallPad의 접속 ssid 와 pw, mode state를 기록하는 구조체
 * 
 */
typedef struct StWallPad
{
  char  sInitChk  [ nMAX_INIT_CHK ];  
  eMODE eMode;             
  char  sSsId   [ nMAX_NAME ];
  char  sSsPass   [ nMAX_NAME ];
  
} WallPad, *pWallPad;

//////////////////////////////////////////////////////////////
// 주변 AP 탐색 모드 시 사용
#define nMAX_SEARCH_AP    30

/**
 * @brief 주변 검색된 AP리스트의 정보를 담는 구조체 (암호화 여부와 수신세기를 포함)
 * 
 */
typedef struct StApList
{
  String      sSsid;
  signed int  nRssi;
  bool        bEncry;
} ApList, *pApList;

/**
 * @brief 업로드가 너무 자주일어나 과금에 영향을 주므로,@n
 * 마지막 업로드의 checksum 과 온도정보를 저장하여, aws 에 업로드 주기를 조절하는 구조체
 * 
 */
typedef struct StLastSend
{
  char        cFcu;
  char        cErv;
  char        cTemp;
  char        cSensor;
}LastSend, *pLastSend;


