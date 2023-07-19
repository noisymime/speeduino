#ifndef INIT_H
#define INIT_H

void initialiseAll(void);
void initialiseTriggers(void);
void setPinMapping(byte boardID);
void changeHalfToFullSync(void);
void changeFullToHalfSync(void);

#define VSS_USES_RPM2() ((configPage2.vssMode > 1) && (pinVSS == pinTrigger2) && !BIT_CHECK(decoderState, BIT_DECODER_HAS_SECONDARY)) // VSS is on the same pin as RPM2 and RPM2 is not used as part of the decoder
#define FLEX_USES_RPM2() ((configPage2.flexEnabled > 0) && (pinFlex == pinTrigger2) && !BIT_CHECK(decoderState, BIT_DECODER_HAS_SECONDARY)) // Same as above, but for Flex sensor

#endif