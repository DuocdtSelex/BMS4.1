#include "nfc.h"
#include "trf79x0.h"
#include "board.h"
#include "mcu.h"
#include "nfc_controller.h"
#include "nfc_config.h"
#include "debug_print.h"

/**************************************************************************/

#define NFC_TIMEOUT_INTERVAL 5
NFC_LINK_ERROR nfc_last_error=NFC_LINK_OK;

NFC_New_Package_Handle nfc_new_package_handle;


bool g_bEnableAutoSDD;
bool g_bExtAmplifier;
bool g_bTRF5VSupply;
tTRF79x0_Version g_eTRFVersion;
bool g_bSupportCertification;
uint16_t g_ui16ListenTime;

//#if (NFC_PEER_2_PEER_INITIATOR_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED)
t_sNfcP2PMode g_sP2PSupportedModes;
t_sNfcP2PCommBitrate g_sP2PSupportedTargetBitrates;
t_sNfcP2PCommBitrate g_sP2PSupportedInitiatorBitrates;
t_sNfcDEP_P2PSetup g_sP2PSetupOptions;
uint8_t g_ui8NfcDepInitiatorDID;
//#endif

/***************************************************************************/

tNfcState eTempNFCState;
tNfcState eCurrentNFCState;

// CE Variables
t_sNfcCEMode sCEMode;

//#if (NFC_PEER_2_PEER_INITIATOR_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED)
// Peer to Peer TX Variables
uint8_t tx_buffer[255]={0};
uint32_t ui32PacketRemaining;
uint8_t ui8TXBytes;
uint16_t ui16TxIndex;
uint32_t ui32PacketLength;
uint8_t * pui8NdefPointer;
uint8_t ui8FragmentSize;
char pcBytesReceivedString[5];

// Bytes Received from Peer to Peer
uint16_t ui16BytesReceived = 0x00;

// Peer to peer RX Status
tNfcP2PRxStatus sP2PRxStatus;
//#endif

t_sNfcP2PMode sP2PMode;
t_sNfcP2PCommBitrate sP2PBitrate;

// Reader/Writer RX Status
t_sNfcRWMode sRWMode;
t_sNfcRWCommBitrate sRWBitrate;
//#if (NFC_PEER_2_PEER_INITIATOR_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED)
t_sNfcP2PMode g_sP2PSupportedModes;
t_sNfcP2PCommBitrate g_sP2PSupportedTargetBitrates;
t_sNfcP2PCommBitrate g_sP2PSupportedInitiatorBitrates;
t_sNfcDEP_P2PSetup g_sP2PSetupOptions;
uint8_t g_ui8NfcDepInitiatorDID;
//#endif

static void nfc_set_technology(void);
static void NFC_initIDs(void);
static void nfc_state_change_handle();
static void nfc_p2p_send_message_segment(const uint8_t* message_data);
static uint16_t ndef_build_text_message(const uint8_t* payload,const uint8_t* id,uint8_t* msg);
static bool is_nfc_link_ready();
static bool nfc_p2p_read_all();

void nfc_link_setup(){
	//Enable interrupts globally
	__enable_irq();

	// Initialize TRF7970
	TRF79x0_init();
	TRF79x0_idleMode();

	// Initialize the NFC Controller
	NFC_init();

	// This function will configure all the settings for each protocol
	nfc_set_technology();

	// Initialize IDs for NFC-A, NFC-B and NFC-F
	NFC_initIDs();
}

bool nfc_open_link(){

	if(is_nfc_link_ready()==false){
		nfc_last_error=NFC_ACTIVE_TIMEOUT;
        return false;
	}
    return true;
}

static bool is_nfc_link_ready(){

	eTempNFCState = NFC_run();
	if(eTempNFCState == NFC_DATA_EXCHANGE_PROTOCOL){
        return true;
	}
	if(eCurrentNFCState != eTempNFCState)
	{
		nfc_state_change_handle();
	}
    return false;
}

static void nfc_state_change_handle(){
	if(eCurrentNFCState != NFC_TARGET_WAIT_FOR_ACTIVATION
		&& eCurrentNFCState != NFC_STATE_IDLE
		&& (eTempNFCState == NFC_PROTOCOL_ACTIVATION
			|| eTempNFCState == NFC_DISABLED))
	{
		eCurrentNFCState = eTempNFCState;

#if (NFC_CARD_EMULATION_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED || NFC_PEER_2_PEER_INITIATOR_ENABLED)
		// Initialize IDs for NFC-A, NFC-B and NFC-F
		NFC_initIDs();
#endif
#if (NFC_PEER_2_PEER_INITIATOR_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED)
		ui16BytesReceived = 0;
		ui32PacketRemaining = 0;
#endif
	}
	else{
		eCurrentNFCState = eTempNFCState;
	}
}

void nfc_wait_for_response(){

    uint32_t timeout_counter=0;
    bool is_finish=false;

    while(timeout_counter < NFC_TIMEOUT_INTERVAL){
        eTempNFCState = NFC_run();
	    if(eTempNFCState == NFC_DATA_EXCHANGE_PROTOCOL){
	    	is_finish=nfc_p2p_read_all();
            if(is_finish==true){
	            nfc_last_error=NFC_LINK_OK;
                return;
            }
	    }

	    if(eCurrentNFCState != eTempNFCState)
	    {
	    	nfc_state_change_handle();
	    }
        timeout_counter++;
    }
   	nfc_last_error=NFC_RESPONSE_TIMEOUT;
}

void nfc_sends(const uint8_t* data,const uint16_t len){

	uint32_t timeout_counter=0;
	ui16TxIndex = 0x00;
	ui32PacketLength =ndef_build_text_message(data,NULL,tx_buffer);
	ui32PacketRemaining = ui32PacketLength;
    pui8NdefPointer = tx_buffer;
    timeout_counter=0;
	while(timeout_counter < NFC_TIMEOUT_INTERVAL){

        eTempNFCState = NFC_run();
	    if(eTempNFCState == NFC_DATA_EXCHANGE_PROTOCOL){
		    nfc_p2p_send_message_segment(tx_buffer);
		    if(ui32PacketRemaining ==0){
	            nfc_last_error=NFC_LINK_OK;
                return;
		    }
	    }

	    if(eCurrentNFCState != eTempNFCState)
	    {
	    	nfc_state_change_handle();
	    }

	    timeout_counter++;
	}

	nfc_last_error=NFC_TRANSMIT_TIMEOUT;
}

void nfc_sync_data(void){

}


NFC_LINK_ERROR nfc_get_error(){
     return nfc_last_error;
}

static void nfc_p2p_send_message_segment(const uint8_t* message){

	bool is_start_segment=true;
	if(ui32PacketRemaining >0){

		if(ui32PacketRemaining < LLCP_MIU)
		{
			ui8FragmentSize = (uint8_t) ui32PacketRemaining;
		}
		else
		{
			ui8FragmentSize = LLCP_MIU;
		}

		if(ui16TxIndex==0){
			is_start_segment=true;
		}
		else{
			is_start_segment=false;
		}

		ui8TXBytes = NFC_P2P_sendNdefPacket(pui8NdefPointer,
				is_start_segment,ui8FragmentSize,ui32PacketLength);

		if(ui8TXBytes)
		{
			ui32PacketRemaining = ui32PacketRemaining - (uint16_t) (ui8TXBytes);
			ui16TxIndex = ui16TxIndex + (uint16_t) ui8TXBytes;
            pui8NdefPointer+= ui8TXBytes;
		}
	}
}


uint8_t* nfc_get_received_buffer(){
    return sP2PRxStatus.pui8RxDataPtr;
}

uint16_t nfc_get_received_buffer_len(){
    return ui16BytesReceived;
}

void nfc_flush_rx_buffer(){
    ui16BytesReceived=0;
}


/* fill ndef header and payload into buffer to send */
static uint16_t ndef_build_text_message(const uint8_t* payload,const uint8_t* id,uint8_t* msg){

    uint8_t header_flags=0;
    uint16_t payload_len=strlen((char*)payload);
    uint8_t * p_payload_field=NULL;

    header_flags |= MB_FLAG_BIT;
    header_flags += 0x01; /* TNF=0x01 : Text record */

    if(payload_len <= 255){ /* short record */
        header_flags |= SR_FLAG_BIT;
        header_flags |= ME_FLAG_BIT;
    }else{
        header_flags &= ~SR_FLAG_BIT;
        header_flags &= ~ME_FLAG_BIT;
    }

    if(id==NULL){
       /* ID field is not included */
        header_flags &= ~IL_FLAG_BIT;
    }else{

    }

    msg[0]= header_flags;
    msg[1]= 1;
    msg[2]=payload_len+3;
    msg[3]=(uint8_t)'T';
    msg[4]=0x02; /* language length */
    msg[5]=(uint8_t)'e';
    msg[6]=(uint8_t)'n';
    p_payload_field = msg+7;

    while(*payload){
        *p_payload_field = *payload;
        p_payload_field++;
        payload++;
    }
    *p_payload_field= '\0';
    return payload_len + 7;
}

static bool nfc_p2p_read_all(){

	if(NFC_P2P_getModeStatus(&sP2PMode,&sP2PBitrate)){
	    sP2PRxStatus = NFC_P2P_getReceiveState();
	    if(sP2PRxStatus.sDataReceivedStatus != RECEIVED_NO_FRAGMENT){
	    	ui16BytesReceived = sP2PRxStatus.ui16DataReceivedLength
                + ui16BytesReceived;
	    	// Check if the last packet was received completely
	    	if((uint16_t) sP2PRxStatus.ui32PacketSize == ui16BytesReceived)
	    	{
                return true;
	    	}
	    }
    }

    return false;
}

static void nfc_set_technology(void)
{
#if (NFC_PEER_2_PEER_INITIATOR_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED)
	g_sP2PSupportedModes.ui8byte = 0x00;
	g_sP2PSupportedTargetBitrates.ui8byte = 0x00;
	g_sP2PSupportedInitiatorBitrates.ui8byte = 0x00;
	g_sP2PSetupOptions.ui8byte = 0x00;
#endif

#if NFC_CARD_EMULATION_ENABLED
#endif

#if NFC_READER_WRITER_ENABLED
#endif

	// Set the TRF7970 Version being used
	g_eTRFVersion = TRF7970_A;

	// External Amplifer (disconnected by default)
	g_bExtAmplifier = false;

	// Configure TRF External Amplifier for the transceiver
	TRF79x0_setExtAmplifer(g_bExtAmplifier);

	// Configure TRF Power Supply (5V = true, 3V = false)
	g_bTRF5VSupply = false;

	// Configure TRF Power Supply
	TRF79x0_setPowerSupply(g_bTRF5VSupply);

	// Milliseconds the NFC stack will be in listen mode
	g_ui16ListenTime =37;

	// Set the time the NFC stack will be with the RF field disabled (listen mode)
	NFC_setListenTime(g_ui16ListenTime);

	// Enable (1) or disable (0) the Auto SDD Anti-collision function of the TRF7970A
	g_bEnableAutoSDD = 0;

#if (NFC_PEER_2_PEER_INITIATOR_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED)

#ifdef BMS_NFC
	// Enable Peer 2 Peer Supported Modes
	g_sP2PSupportedModes.bits.bTargetEnabled = 1;
	g_sP2PSupportedModes.bits.bInitiatorEnabled = 0;

	// Set P2P Supported Bit Rates - Target mode
	g_sP2PSupportedTargetBitrates.bits.bPassive106kbps = 0;
	// Set P2P Supported Bit Rates - Initiator mode
	g_sP2PSupportedInitiatorBitrates.bits.bPassive106kbps = 0;
#endif

#ifdef VES_NFC
	// Enable Peer 2 Peer Supported Modes
	g_sP2PSupportedModes.bits.bTargetEnabled = 1;
	g_sP2PSupportedModes.bits.bInitiatorEnabled = 1;

	g_sP2PSupportedInitiatorBitrates.bits.bPassive106kbps = 1;
	g_sP2PSupportedTargetBitrates.bits.bPassive106kbps = 1;

#endif
	g_sP2PSupportedTargetBitrates.bits.bPassive212kbps = 0;
	g_sP2PSupportedTargetBitrates.bits.bPassive424kbps = 1;
	g_sP2PSupportedInitiatorBitrates.bits.bPassive212kbps = 0;
	g_sP2PSupportedInitiatorBitrates.bits.bPassive424kbps = 0;

	g_sP2PSupportedTargetBitrates.bits.bActive106kbps = 0;
	g_sP2PSupportedTargetBitrates.bits.bActive212kbps = 0;
	g_sP2PSupportedTargetBitrates.bits.bActive424kbps = 0;
	g_sP2PSupportedInitiatorBitrates.bits.bActive106kbps = 0;
	g_sP2PSupportedInitiatorBitrates.bits.bActive212kbps = 0;
	g_sP2PSupportedInitiatorBitrates.bits.bActive424kbps = 0;




	// Certification Config Start //

	// Enable (1) or disable (0) Wave 1 NFC Forum Certification functionality
	// Note: Enabling this feature can affect interoperability with NFC Devices that are not certified.
	g_bSupportCertification = 0;

	// Allows for Customization of the DID (Device Identification) number when in initiator mode
	g_ui8NfcDepInitiatorDID = 0x00;

	// Enable LLCP
	g_sP2PSetupOptions.bits.bP2PSupportLLCP = 1;

	// Enable Loopback
	g_sP2PSetupOptions.bits.bP2PSupportLoopback = 0;

	// Specify maximum number of timeouts and protocol errors allowed before resetting
	g_sP2PSetupOptions.bits.ui3P2PMaxTimeouts = 2;
	g_sP2PSetupOptions.bits.ui3P2PMaxProtocolErrors = 2;
#endif

#if NFC_CARD_EMULATION_ENABLED

#endif

#if NFC_READER_WRITER_ENABLED
#endif

#if (NFC_PEER_2_PEER_INITIATOR_ENABLED || NFC_PEER_2_PEER_TARGET_ENABLED)
	// Configure Peer 2 Peer functions for the correct modes and communication bitrates
	NFC_P2P_configure(g_sP2PSupportedModes,g_sP2PSupportedTargetBitrates,g_sP2PSupportedInitiatorBitrates);

	// Configure NFC DEP functions including passing the DID
	NFCDEP_configure_P2P(g_sP2PSetupOptions,g_bSupportCertification,g_ui8NfcDepInitiatorDID);
#endif

#if NFC_CARD_EMULATION_ENABLED
#endif


#if NFC_READER_WRITER_ENABLED
#endif

	// Set the Auto SDD flag within nfc_a.c
	NFC_A_setAutoSDD(g_bEnableAutoSDD);

	// Set the current TRF version within trf79x0.c
	TRF79x0_setVersion(g_eTRFVersion);

	// Set Certification Support for all Protocols - Required for NFC Forum Certification
	NFC_setSupportCertification(g_bSupportCertification);

	// Set Test Enable flag within trf79x0.c - Required for NFC Forum Certification
	TRF79x0_testFlag(g_bSupportCertification);

}

static void NFC_initIDs(void)
{
	// NFC ID's
	uint8_t pui8NfcAId[10] = {0x08,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};	// Generic ISO14443 T4TA Tag
	uint8_t pui8NfcBId[4] = {0x08,0x0A, 0xBE,0xEF};	// Generic ISO14443 T4TB Tag
	uint8_t pui8NfcFId[8] = {0x01,0xFE,0x88,0x77,0x66,0x55,0x44,0x33};	// Type F ID for P2P

	// Set the NFC Id's for Type A, Type B, and Type F
	NFC_A_setNfcAId(pui8NfcAId,4);
	NFC_B_setNfcBId(pui8NfcBId,4);
	NFC_F_setNfcId2(pui8NfcFId,8);
}

void nfc_register_received_handle(NFC_New_Package_Handle handle){
	nfc_new_package_handle=handle;
}
