/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef TPL_H
#define TPL_H

enum OPCODE {OPCODE_TL,OPCODE_TR,
             OPCODE_FWD,OPCODE_REV,OPCODE_SPEED,OPCODE_INVERT_DIRECTION,
             OPCODE_RESERVE,OPCODE_FREE,
             OPCODE_AT,OPCODE_AFTER,OPCODE_SET,OPCODE_RESET,
             OPCODE_IF,OPCODE_IFNOT,OPCODE_ENDIF,OPCODE_IFRANDOM,
             OPCODE_DELAY,OPCODE_DELAYMINS,OPCODE_RANDWAIT,
             OPCODE_FON, OPCODE_FOFF,
             OPCODE_RED,OPCODE_GREEN,OPCODE_AMBER,
             OPCODE_PAD,OPCODE_FOLLOW,OPCODE_ENDROUTE,
             OPCODE_PROGTRACK,OPCODE_READ_LOCO1,OPCODE_READ_LOCO2,
             OPCODE_SCHEDULE,OPCODE_SETLOCO,
             OPCODE_PAUSE, OPCODE_RESUME, 
             OPCODE_ROUTE,OPCODE_ENDROUTES
             };
 

 class TPL {
   public:
    static void begin();
    static void loop();
    TPL(byte route);
    ~TPL();
    static void readLocoCallback(int cv);
  private: 
    static void ComandFilter(Print * stream, byte & opcode, byte & paramCount, int p[]);
    static bool parseSlash(Print * stream, byte & paramCount, int p[]) ;

    static int locateRouteStart(short _route);
    static int progtrackLocoId;
    static TPL * loopTask;
    static TPL * pausingTask;
    void delayMe(int millisecs);
    void driveLoco(byte speedo);
    bool readSensor(short id);
    void skipIfBlock();
    bool readLoco();
    void showManual();
    void showProg(bool progOn);
    bool doManual();
    void loop2();          
    

  static const  PROGMEM  byte RouteCode[];
 
 
 // Local variables
    int progCounter;    // Byte offset of next route opcode in ROUTES table
    unsigned long delayStart; // Used by opcodes that must be recalled before completing
    unsigned long waitAfter; // Used by OPCODE_AFTER
    unsigned int  delayTime;
    int loco;
    bool forward;
    bool invert;
    int speedo;
    TPL *next;   // loop chain 
};
#endif
