/*
 * wav.h
 *
 *  Created on: Jun 9, 2023
 *      Author: Admins
 */

#ifndef INC_WAV_H_
#define INC_WAV_H_

#pragma pack(1)

typedef struct Wav_Header {
  char     riff[4];
  uint32_t fileSize;
  char     fileTypeHeader[4];
  char     formatChunkMarker[4];
  uint32_t formatChunkLength;
  uint16_t vfmt;
  uint16_t channels;
  uint32_t sampleFreq;
  uint32_t sampleBytesPerSecond;
  uint16_t blkSize;
  uint16_t bitsPerSample;
  char     dataChunkHeader[4];
  uint32_t dataChunkLength;
}WAV;


#endif /* INC_WAV_H_ */
