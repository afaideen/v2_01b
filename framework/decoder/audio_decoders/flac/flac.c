
#include <stdio.h>
#include <stdlib.h>
#include "FLAC/stream_decoder.h"
#include "decoder.h"

typedef struct 
{
    FLAC__uint8 channelLeft;
    FLAC__uint8 channelRight;
}DecodedData;

typedef struct {
	SYS_FS_HANDLE file;
	FLAC__bool ignore_errors;
	FLAC__bool error_occurred;
    FLAC__bool eof;
    uint16_t *decodebuf;
    uint16_t *decodesize;
    FLAC__StreamDecoder *decoder;
    FLAC__uint64 total_samples;
    FLAC__uint32 sample_rate;
    FLAC__uint32 channels;
    FLAC__uint32 bps;
    FLAC__uint32 min_blocksize;
    FLAC__uint32 max_blocksize;
    FLAC__uint32 min_framesize;
    FLAC__uint32 max_framesize;
    FLAC__uint64 flacfilesize_;
    FLAC__uint32 bitrate;
} StreamDecoderClientData;

static StreamDecoderClientData FlacData;

static FLAC__StreamDecoderTellStatus stream_decoder_tell_callback_(const FLAC__StreamDecoder *decoder, FLAC__uint64 *absolute_byte_offset, void *client_data)
{
	StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;

	if(0 == dcd) {
		return FLAC__STREAM_DECODER_TELL_STATUS_ERROR;
	}

	if(dcd->error_occurred)
		return FLAC__STREAM_DECODER_TELL_STATUS_ERROR;

	*absolute_byte_offset = SYS_FS_FileTell(dcd->file);

	if(*absolute_byte_offset < 0) {
		dcd->error_occurred = true;
		return FLAC__STREAM_DECODER_TELL_STATUS_ERROR;
	}

	return FLAC__STREAM_DECODER_TELL_STATUS_OK;
}

static FLAC__StreamDecoderLengthStatus stream_decoder_length_callback_(const FLAC__StreamDecoder *decoder, FLAC__uint64 *stream_length, void *client_data)
{
	StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;

	if(0 == dcd) {
		return FLAC__STREAM_DECODER_LENGTH_STATUS_ERROR;
	}

	if(dcd->error_occurred)
		return FLAC__STREAM_DECODER_LENGTH_STATUS_ERROR;

	*stream_length = (FLAC__uint64)dcd->flacfilesize_;
	return FLAC__STREAM_DECODER_LENGTH_STATUS_OK;
}

static FLAC__bool stream_decoder_eof_callback_(const FLAC__StreamDecoder *decoder, void *client_data)
{
	StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;
    bool eof;
    
	if(0 == dcd) {
		return true;
	}

	if(dcd->error_occurred)
		return true;

	dcd->eof = eof = SYS_FS_FileEOF(dcd->file);
    return eof;
}

static void stream_decoder_error_callback_(const FLAC__StreamDecoder *decoder, FLAC__StreamDecoderErrorStatus status, void *client_data)
{
	StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;

	if(0 == dcd) {
		return;
	}

	if(!dcd->ignore_errors) {
		dcd->error_occurred = true;
	}
}


static FLAC__StreamDecoderSeekStatus stream_decoder_seek_callback_(const FLAC__StreamDecoder *decoder, FLAC__uint64 absolute_byte_offset, void *client_data)
{
	StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;

	if(0 == dcd) {
		return FLAC__STREAM_DECODER_SEEK_STATUS_ERROR;
	}

	if(dcd->error_occurred)
		return FLAC__STREAM_DECODER_SEEK_STATUS_ERROR;

	if(SYS_FS_FileSeek(dcd->file, absolute_byte_offset, SEEK_SET) < 0) {
		dcd->error_occurred = true;
		return FLAC__STREAM_DECODER_SEEK_STATUS_ERROR;
	}

	return FLAC__STREAM_DECODER_SEEK_STATUS_OK;
}

static FLAC__StreamDecoderReadStatus stream_decoder_read_callback_(const FLAC__StreamDecoder *decoder, FLAC__byte buffer[], size_t *bytes, void *client_data)
{
	StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;
	const size_t requested_bytes = *bytes;

	if(0 == dcd) {
		return FLAC__STREAM_DECODER_READ_STATUS_ABORT;
	}

	if(dcd->error_occurred)
		return FLAC__STREAM_DECODER_READ_STATUS_ABORT;

	if(SYS_FS_FileEOF(dcd->file)) {
		*bytes = 0;
		return FLAC__STREAM_DECODER_READ_STATUS_END_OF_STREAM;
	}
	else if(requested_bytes > 0) {
		*bytes = SYS_FS_FileRead(dcd->file, buffer, requested_bytes);
		if(*bytes == 0) {
			if(SYS_FS_FileEOF(dcd->file))
				return FLAC__STREAM_DECODER_READ_STATUS_END_OF_STREAM;
			else
				return FLAC__STREAM_DECODER_READ_STATUS_ABORT;
		}
		else {
			return FLAC__STREAM_DECODER_READ_STATUS_CONTINUE;
		}
	}
	else
		return FLAC__STREAM_DECODER_READ_STATUS_ABORT; /* abort to avoid a deadlock */
}

void stream_decoder_metadata_callback_(const FLAC__StreamDecoder *decoder, const FLAC__StreamMetadata *metadata, void *client_data)
{
    StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;
	if(metadata->type == FLAC__METADATA_TYPE_STREAMINFO) {
		dcd->total_samples = metadata->data.stream_info.total_samples;
		dcd->sample_rate = metadata->data.stream_info.sample_rate;
		dcd->channels = metadata->data.stream_info.channels;
		dcd->bps = metadata->data.stream_info.bits_per_sample;
        dcd->min_blocksize=metadata->data.stream_info.min_blocksize;
        dcd->max_blocksize = metadata->data.stream_info.max_blocksize;
        dcd->min_framesize = metadata->data.stream_info.min_framesize;
        dcd->max_framesize = metadata->data.stream_info.max_framesize;
	}
}

FLAC__StreamDecoderWriteStatus stream_decoder_write_callback_(const FLAC__StreamDecoder *decoder, const FLAC__Frame *frame, const FLAC__int32 * const buffer[], void *client_data)
{
    StreamDecoderClientData *dcd = (StreamDecoderClientData*)client_data;
	uint32_t i = 0, j = 0;

	if(dcd->total_samples == 0) {
		return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
	}
    
//	if(dcd->channels != 2 || dcd->bps != 16) {
//		return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
//	}
//    
//	if(frame->header.channels != 2) {
//		return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
//	}
    
	if(buffer[0] == NULL) {
		return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
	}
	if(buffer[1] == NULL) {
		return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
	}

	/* write decoded PCM samples */
    for(i = 0, j=0; i < frame->header.blocksize; i++, j++) {
       dcd->decodebuf[j] = (FLAC__uint16) buffer[0][i];
    }

    *(dcd->decodesize) = frame->header.blocksize * 2;

	return FLAC__STREAM_DECODER_WRITE_STATUS_CONTINUE;
}

bool FLAC_Initialize (SYS_FS_HANDLE fhandle, char *fname)
{
	FLAC__StreamDecoderInitStatus init_status;
    FLAC__bool ok = true;
    
    FlacData.file = fhandle;
	FlacData.ignore_errors = false;
	FlacData.error_occurred = false;
    FlacData.eof = false;
    
    if((FlacData.decoder = FLAC__stream_decoder_new()) == NULL) {
		return false;
	}
    
    (void)FLAC__stream_decoder_set_md5_checking(FlacData.decoder, false);
    
    init_status = FLAC__stream_decoder_init_stream(FlacData.decoder, 
                                                    stream_decoder_read_callback_,
                                                    stream_decoder_seek_callback_,
                                                    stream_decoder_tell_callback_, 
                                                    stream_decoder_length_callback_, 
                                                    stream_decoder_eof_callback_, 
                                                    stream_decoder_write_callback_, 
                                                    stream_decoder_metadata_callback_, 
                                                    stream_decoder_error_callback_, 
                                                    &FlacData);
	if(init_status != FLAC__STREAM_DECODER_INIT_STATUS_OK) {
		return false;
	}
    
   
    if(!FLAC__stream_decoder_process_until_end_of_metadata(FlacData.decoder))
		return false;
    
    //bitrate = (sample rate) * (bit depth) * (number of channels) = kbits per second
    FlacData.bitrate = (FlacData.sample_rate * FlacData.bps * FlacData.channels);
    
    return ok;
}

bool FLAC_Decoder( uint8_t *input, uint16_t inSize, uint16_t *read, uint8_t *output, uint16_t *written )
{
    if(FlacData.decoder != NULL)
    {
        FlacData.decodebuf = (uint16_t *)output;
        FlacData.decodesize = written;
        return FLAC__stream_decoder_process_single(FlacData.decoder);
    }
    return false;
}

bool isFLACdecoder_enabled()
{
    return true;
}

uint8_t FLAC_GetChannels(){
    return 1; //FlacData.channels;
}

void FLAC_RegisterDecoderEventHandlerCallback(DecoderEventHandlerCB fptr)
{
    return;
}

int32_t FLAC_GetBitRate()
{
    return FlacData.bitrate;
}
int32_t FLAC_GetSamplingRate() 
{
    return FlacData.sample_rate;
}

int32_t FLAC_GetBlockSize()
{
    FLAC__StreamDecoderState state;
    if(FlacData.decoder != NULL)
    {
        state = FLAC__stream_decoder_get_state(FlacData.decoder);
        if((state == FLAC__STREAM_DECODER_END_OF_STREAM) || (state == FLAC__STREAM_DECODER_ABORTED) )
        {
            FLAC__stream_decoder_delete(FlacData.decoder);
            return -1;
        }
        else
        {
            return FlacData.max_blocksize;
        }            
    }
    return -1; 
} 

void FLAC_Cleanup() 
{
    if(FlacData.decoder != NULL)
        FLAC__stream_decoder_delete(FlacData.decoder);
}

