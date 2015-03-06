//
//  ScnVc.m
//  model3d
//
//  Created by Andreas Hauenstein on 2015-01-12.
//  Copyright (c) 2015 AHN. All rights reserved.
//

#import "AppDelegate.h"
#import "ScnVc.h"
#import "Util.h"
#import <SceneKit/SceneKit.h>
#import <GLKit/GLKit.h>
#import <CoreFoundation/CoreFoundation.h>
#import <sys/socket.h>
#import <netinet/in.h>

#define MYPORT 2718

#define DEG(x) ((x)*(180.0/M_PI))
#define RAD(x) ((x)*(M_PI/180.0))


#define APP ((AppDelegate*) [NSApplication sharedApplication].delegate)

#define RGB(rgbValue) [NSColor \
colorWithRed:((float)((rgbValue & 0xFF0000) >> 16))/255.0 \
green:((float)((rgbValue & 0xFF00) >> 8))/255.0 \
blue:((float)(rgbValue & 0xFF))/255.0 alpha:1.0]

//---------------------
@interface ScnVc ()
//---------------------

@property (weak) IBOutlet NSTextField *lbIP;
@property (weak) IBOutlet NSTextField *lbPort;

@property SCNNode *brickNode;
@property float corrAngle;

// Socket stuff
@property CFSocketRef socket;
@property NSInputStream *sockistream;
@property NSOutputStream *sockostream;

// Data from the sensoplex dump file
@property NSArray *sensoData;


@end // ScnVc

//---------------------
@implementation ScnVc
//---------------------


#pragma mark Lifecycle
//---------------------------------------------------
- (void)awakeFromNib
//---------------------------------------------------
{
    APP.scnVc = self;
    
    // Use SceneKit to show a 3D brick
    //======================================
    SCNView *sceneView = (SCNView *) self.view;
    sceneView.backgroundColor = [NSColor grayColor];
    //sceneView.allowsCameraControl = true;

    // Create the scene and get the root
    sceneView.scene = [SCNScene scene];
    SCNNode *root = sceneView.scene.rootNode;
    
//    // camera
//    SCNCamera *camera = [SCNCamera new];
//    camera.xFov = 40;
//    camera.yFov = 40;
//    
//    _cameraNode = [SCNNode new];
//    _cameraNode.camera = camera;
//    _cameraNode.position = SCNVector3Make(0,0,4);
//    
//    [root addChildNode:_cameraNode];
    
    // Create the brick geometry and node
    SCNBox *brickGeom = [SCNBox
                         boxWithWidth:2.2
                         height:0.1
                         length:1.0
                         chamferRadius:0.05];
    _brickNode = [SCNNode nodeWithGeometry:brickGeom];
    
    NSColor *c1 = RGB (0x6a7dc1);
    NSColor *c2 = RGB (0x6d6f76);
    NSColor *c3 = RGB (0xae7b7b);
    NSColor *c4 = RGB (0x8ba782);
    
    NSImage *iphoneFront = [NSImage imageNamed:@"iphone_front.png"];
    NSImage *iphoneBack = [NSImage imageNamed:@"iphone_back.png"];
    
    SCNMaterial *c1Material              = [SCNMaterial material];
    c1Material.diffuse.contents          = c1;

    SCNMaterial *c2Material                = [SCNMaterial material];
    c2Material.diffuse.contents          = c2;
    
    SCNMaterial *c3Material               = [SCNMaterial material];
    c3Material.diffuse.contents          = c3;
    
    SCNMaterial *c4Material             = [SCNMaterial material];
    c4Material.diffuse.contents          = c4;
    
    SCNMaterial *frontMaterial             = [SCNMaterial material];
    frontMaterial.diffuse.contents          = iphoneFront;
    
    SCNMaterial *backMaterial            = [SCNMaterial material];
    backMaterial.diffuse.contents          = iphoneBack;
    
    _brickNode.geometry.materials =
    @[c1Material,  c2Material, c3Material,
      c4Material, frontMaterial, backMaterial];
    [root addChildNode:_brickNode];
    
    // Init IP and Port labels
    _lbPort.stringValue = nsprintf (@"%d", MYPORT);
    _lbIP.stringValue = getIP();
    
    [self listenOnPort:MYPORT];
} // awakeFromNib

#pragma mark Button Callbacks

//--------------------------------
- (IBAction)btn10045:(id)sender
//--------------------------------
// Rotate by 45 degrees around x-axis
{
//    if ([_sockistream hasBytesAvailable]) {
//        char buf[1000];
//        [_sockistream read:(uint8_t*)buf maxLength:1000];
//    }
    // Rotation before
    SCNQuaternion ori = self.brickNode.rotation;
    // Change in rotation
    SCNQuaternion rot = SCNVector4Make (1, 0, 0, M_PI/4.0);
    // Rotation after
    SCNQuaternion newOri = SCNQuaternionMultiply (ori, rot);
    // Without this, the brick will snap back at the end of animation
    _brickNode.rotation = newOri;
    NSLog (@"axyz: %.2f, %.2f, %.2f, %.2f",DEG(newOri.w),newOri.x,newOri.y,newOri.z);
} // btn10045

//-------------------------------------
- (IBAction)btn01045:(id)sender
//-------------------------------------
// Rotate by 45 degrees around y-axis
{
    SCNQuaternion ori = self.brickNode.rotation;
    SCNQuaternion rot = SCNVector4Make (0, 1, 0, M_PI/4.0);
    SCNQuaternion newOri = SCNQuaternionMultiply (ori, rot);
    _brickNode.rotation = newOri;
    NSLog (@"axyz: %.2f, %.2f, %.2f, %.2f",DEG(newOri.w),newOri.x,newOri.y,newOri.z);
}

//-------------------------------------
- (IBAction)btn00145:(id)sender
//-------------------------------------
// Rotate by 45 degrees around z-axis
{
    SCNQuaternion ori = self.brickNode.rotation;
    SCNQuaternion rot = SCNVector4Make (0, 0, 1, M_PI/4.0);
    SCNQuaternion newOri = SCNQuaternionMultiply (ori, rot);
    _brickNode.rotation = newOri;
    NSLog (@"axyz: %.2f, %.2f, %.2f, %.2f",DEG(newOri.w),newOri.x,newOri.y,newOri.z);
}

//-------------------------------------
- (IBAction)btnto10045:(id)sender
//-------------------------------------
// Absolute rotation to 1,0,0,45 orientation
{
    //SCNQuaternion newOri = SCNVector4Make (1, 0, 0, M_PI/4.0);
    SCNQuaternion newOri = SCNVector4Make (1, 1, 0,M_PI);
    _brickNode.rotation = newOri;
}

//------------------------------
- (IBAction)btnCorr:(id)sender
//------------------------------
{
    float rot = DEG(eulerPhi(_brickNode.rotation)) - _corrAngle;
    _corrAngle = 90.0 - rot;
 //   _corrAngle += 5.0;
}

//---------------------------------------
- (IBAction)btnReadSensoFile:(id)sender
//---------------------------------------
{
    NSOpenPanel* openDlg = [NSOpenPanel openPanel];
    
    // Enable the selection of files in the dialog.
    [openDlg setCanChooseFiles:YES];
    
    // Disable the selection of directories in the dialog.
    [openDlg setCanChooseDirectories:NO];
    
    // Change "Open" dialog button to "Select"
    [openDlg setPrompt:@"Select"];
    
    // Display the dialog.  If the OK button was pressed,
    // process the first selected file
    if ( [openDlg runModal] == NSOKButton )
    {
        // Get an array containing the full filenames of all
        // files and directories selected.
        NSArray* files = [openDlg URLs];
        if ([files count]) {
            NSURL* infile = files[0];
            [self parseSensoFile:infile];
        }
    }
} // btnReadSensoFile


//--------------------------------
- (IBAction)btnPlay:(id)sender
//--------------------------------
// Replay sensoplex quaternions
{
    const int FREQ = 100;
    if (!_sensoData) {
        [self popup:@"No sensoplex data" title:@"Error"];
        return;
    }
    static int idx;
    if (sender) {
        idx = -1;
    }
    idx++;
    if (idx >= [_sensoData count]) { // done
        [self popup:@"End of Data" title:@"Info"];
        return;
    }
    
    NSDictionary * frame = _sensoData[idx];
    float w = [frame[@"quatw"] floatValue];
    
    // Note how we need to change the axes
    float z = [frame[@"quatx"] floatValue];
    float x = [frame[@"quaty"] floatValue];
    float y = [frame[@"quatz"] floatValue];

    GLKQuaternion glkq;
    glkq.q[0] = x;
    glkq.q[1] = y;
    glkq.q[2] = z;
    glkq.q[3] = w;
    SCNQuaternion newOri = glk2SCN(glkq);
        //SCNQuaternion newOri = SCNVector4Make (x, z, y, theta);
        //        SCNQuaternion newOri = SCNVector4Make (y, x, z, theta);
        //SCNQuaternion newOri = SCNQuaternionNorm (SCNVector4Make (y, z, x, theta));
    _brickNode.rotation = newOri;
    [self performSelector:@selector(btnPlay:) withObject:nil afterDelay:1.0/FREQ];
} // btnPlay


//-------------------------------------
- (void) parseSensoFile:(NSURL *)inf
//-------------------------------------
// Parse a Sensoplex log into memory
{
    NSMutableArray *sensoData = [NSMutableArray new];
    NSString *path = [inf path];
    NSArray *lines = [Util readLinesFromFile:path];
    BOOL headerFound = NO;
    NSMutableArray *columns = [NSMutableArray new];
    for (NSString *line in lines) {
        if (!headerFound) {
            if (nstrstr(line,@"DateTime")) {
                headerFound = YES;
                // Deal with Sensoplex csv quirk
                NSString *header = [Util replaceRegex:@"DateTime"
                                            inString:line
                                          withString:@"Date,Time"];
                NSArray *cols = [header componentsSeparatedByString:@","];
                for (NSString *col in cols) {
                    NSString *column = [trim(col) lowercaseString];
                    column = [Util replaceRegex:@"[\\s]+" inString:column withString:@"_"];
                    [columns addObject:trim(column)];
                }
            }
            continue;
        } // if (!headerFound)
        if ([trim(line) length] == 0) { continue; }
        NSArray *words = [line componentsSeparatedByString:@","];
        NSMutableDictionary *lineDict = [NSMutableDictionary new];
        int i = -1;
        for (NSString *word in words) {
            i++;
            NSString *col = columns[i];
            if (nstrstr(col,@"gyro")  // numbers in q16 fixed format
                || nstrstr(word,@"accel")
                || nstrstr(word,@"euler"))
            {
                double val = [word floatValue];
                val /= 1<<16;
                lineDict[col] = @(val);
            }
            else if (nstrstr(col,@"quat")) { // numbers in q30 fixed format
                double val = [word floatValue];
                val /= 1<<30;
                lineDict[col] = @(val);
            }
            else if (nstrstr(col,@"record")  // integers
                || nstrstr(word,@"type")
                || nstrstr(word,@"timestamp"))
            {
                int val = [word intValue];
                lineDict[col] = @(val);
            }
            else { // all else is a string
                lineDict[col] = trim(word);
            }
        } // for word in words
        [sensoData addObject:[lineDict copy] ];
    } // for line in lines
    _sensoData = [sensoData copy];
} // parseSensoFile()

#pragma mark UI stuff

//-------------------------------
- (void) popup:(NSString *)msg
         title:(NSString *)title
//-------------------------------
{
    NSAlert *alert = [NSAlert new];
    [alert addButtonWithTitle:@"Ok"];
    [alert setMessageText:title];
    [alert setInformativeText:msg];
    [alert setAlertStyle:NSWarningAlertStyle];
    [alert beginSheetModalForWindow:[self.view window]
                      modalDelegate:self
                     //didEndSelector:@selector(alertDidEnd:returnCode:contextInfo:)
                     didEndSelector:nil
                        contextInfo:nil];
}


#pragma mark TCP server

//--------------------------------
- (void) listenOnPort:(int)port
//--------------------------------
{
    // Socket magic from the Apple documentation
    CFSocketContext socketCtxt = {0, (__bridge void *)(self), NULL, NULL, NULL};
    _socket =
    CFSocketCreate (kCFAllocatorDefault
                    ,PF_INET
                    ,SOCK_STREAM
                    ,IPPROTO_TCP
                    ,kCFSocketAcceptCallBack
                    ,handleSocketConnect // callback, see below
                    ,&socketCtxt);
    
    struct sockaddr_in sin;
    memset (&sin, 0, sizeof(sin));
    sin.sin_len = sizeof (sin);
    sin.sin_family = AF_INET; /* Address family */
    sin.sin_port = htons (port);
    sin.sin_addr.s_addr= INADDR_ANY;
    
    CFDataRef sincfd =
    CFDataCreate (kCFAllocatorDefault
                  ,(UInt8 *) &sin
                  ,sizeof (sin));
    
    CFSocketError err =  CFSocketSetAddress (_socket, sincfd);
    if (err != kCFSocketSuccess) {
        [self popup:@"Failed to open port for listening" title:@"Error"];
    }
    CFRelease(sincfd);
    
    // Add socket to runloop
    CFRunLoopSourceRef socketsource =
    CFSocketCreateRunLoopSource (kCFAllocatorDefault
                                 ,_socket
                                 ,0);
    CFRunLoopAddSource (CFRunLoopGetCurrent()
                        ,socketsource
                        ,kCFRunLoopDefaultMode);
    
    
} // listenOnPort

// This function is called by CFSocket when a new connection comes in.
// Magic from
// https://github.com/tuscland/osc-echo-example/blob/master/TCPServer.m
//--------------------------------------------------------------------
static void handleSocketConnect (CFSocketRef socket
                                 ,CFSocketCallBackType type
                                 ,CFDataRef address
                                 ,const void *data
                                 ,void *info)
//--------------------------------------------------------------------
{
    ScnVc *vc = (__bridge ScnVc *)info; // Access to our ViewController class
    
    if (kCFSocketAcceptCallBack == type) {
        // for an AcceptCallBack, the data parameter is a pointer to a CFSocketNativeHandle
        CFSocketNativeHandle nativeSocketHandle = *(CFSocketNativeHandle *)data;
        uint8_t peerName [SOCK_MAXADDRLEN];
        socklen_t namelen = sizeof (peerName);
        NSData *peer = nil;
        
        if (0 == getpeername (nativeSocketHandle, (struct sockaddr *)peerName, &namelen)) {
            peer = [NSData dataWithBytes:peerName length:namelen];
        }
        
        CFReadStreamRef readStream = NULL;
        CFWriteStreamRef writeStream = NULL;
        CFStreamCreatePairWithSocket (kCFAllocatorDefault
                                      ,nativeSocketHandle
                                      ,&readStream
                                      ,&writeStream);
        
        if (readStream && writeStream) {
            if (! CFReadStreamOpen(readStream) || ! CFWriteStreamOpen(writeStream)) {
                [vc popup:@"Failed to open socket streams" title:@"Error"];
            }
            CFReadStreamSetProperty (readStream
                                     ,kCFStreamPropertyShouldCloseNativeSocket
                                     ,kCFBooleanTrue);
            CFWriteStreamSetProperty (writeStream
                                      ,kCFStreamPropertyShouldCloseNativeSocket
                                      ,kCFBooleanTrue);
            [vc handleNewConnectionFromAddress:peer
                                   inputStream:(__bridge NSInputStream *)readStream
                                  outputStream:(__bridge NSOutputStream *)writeStream];
        } else {
            // on any failure, need to destroy the CFSocketNativeHandle
            // since we are not going to use it any more
            close(nativeSocketHandle);
        }
        
//        if (readStream)
//            CFRelease (readStream);
//        
//        if (writeStream)
//            CFRelease (writeStream);
    }
} // handleSocketConnect


//----------------------
- (void) stopListening
//----------------------
{
    CFSocketInvalidate(_socket);
    CFRelease(_socket);
    _socket = NULL;
}

//-------------------------------------------------------------
- (void)handleNewConnectionFromAddress:(NSData *)addr
                           inputStream:(NSInputStream *)istr
                          outputStream:(NSOutputStream *)ostr
//-------------------------------------------------------------
{
    _sockistream = istr;
    _sockostream = ostr;
    [_sockistream setDelegate:self];
    [_sockistream scheduleInRunLoop:[NSRunLoop mainRunLoop]
                            forMode:NSDefaultRunLoopMode];
    //[self write2Client:@"Hello!\n"];
}

//--------------------------------------
- (void) write2Client:(NSString *)msg
//--------------------------------------
{
    const char *cmsg = [msg cStringUsingEncoding:NSUTF8StringEncoding];
    [_sockostream write:(uint8_t *)cmsg
              maxLength:strlen(cmsg)];
}

//--------------------------------------------
- (void) handleClientMessage:(NSString *)msg
//--------------------------------------------
{
    NSArray *lines = splitOnNewLine(msg);
    for (NSString *line in lines) {
        if (!strmatch (line, @"^([0-9.\-]+,){3}[0-9.\-]+$")) { // if not a quaternion
            if ([line length]) {
                NSLog (@"client msg:%@",line);
            }
            return;
        }
        NSArray *words = splitOnComma(line);
        float theta = [words[0] floatValue];
        float x = [words[1] floatValue];
        float y = [words[2] floatValue];
        float z = [words[3] floatValue];
        //SCNQuaternion newOri = SCNVector4Make (x, y, z, theta);
        //SCNQuaternion newOri = SCNVector4Make (x, z, y, theta);
//        SCNQuaternion newOri = SCNVector4Make (y, x, z, theta);
        SCNQuaternion newOri = SCNQuaternionNorm (SCNVector4Make (y, z, x, theta));
        _brickNode.rotation = [self correct:newOri];
        //NSLog(@"axyz: %.2f, %.2f, %.2f, %.2f",DEG(newOri.w),newOri.x,newOri.y,newOri.z);
//        NSLog(@"ftp: %.2f, %.2f, %.2f"
//              ,DEG(eulerPhi(_brickNode.rotation))
//              ,DEG(eulerTheta(_brickNode.rotation))
//              ,DEG(eulerPsi(_brickNode.rotation)));
    }
} // handleClientMessage

//-------------------------------------------------
- (SCNQuaternion) correct:(SCNQuaternion) ori
//-------------------------------------------------
{
    SCNQuaternion corr = SCNVector4Make (0, 1, 0, RAD(_corrAngle));
    return SCNQuaternionMultiply (corr,ori);
} // correct

// Stream events from client. _sockistream uses this method as callback.
//------------------------------------------------------------------------
- (void) stream:(NSStream *)stream handleEvent:(NSStreamEvent)eventCode
//------------------------------------------------------------------------
{
    NSMutableData *data = [NSMutableData new];
    switch(eventCode) {
        case NSStreamEventHasBytesAvailable: {
            uint8_t buf[1024];
            unsigned long len = 0;
            len = [(NSInputStream *)stream read:buf maxLength:1024];
            if(len) {
                [data appendBytes:(const void *)buf length:len];
                int bytesRead;
                bytesRead += len;
            } else {
                NSLog(@"No data.");
            }
            NSString *str =
            [[NSString alloc] initWithData:data
                                  encoding:NSUTF8StringEncoding];
            //[self popup:str title:@"Server Message"];
            [self handleClientMessage:str];
            break;
        }
        case NSStreamEventErrorOccurred: {
            [_sockistream close];
            [_sockostream close];
            _sockistream = NULL;
            _sockostream = NULL;
            NSError *theError = [stream streamError];
            NSString *msg = nsprintf (@"Error %d: %@"
                                      ,[theError code]
                                      ,[theError localizedDescription]);
            [self popup:msg title: @"Stream Error"];
            break;
        }
        case NSStreamEventNone: {
            NSLog (@"NSStreamEventNone");
            break;
        }
        case NSStreamEventOpenCompleted: {
            //[self ledGreen];
            NSLog (@"NSStreamEventOpenCompleted");
            break;
        }
        case NSStreamEventHasSpaceAvailable: {
            NSLog (@"NSStreamEventHasSpaceAvailable");
            break;
        }
        case NSStreamEventEndEncountered: {
            NSLog (@"NSStreamEventEndEncountered");
            break;
        }
        default: {
            [self popup:@"unknown stream event" title: @"Stream Error"];
        }
    } // switch
} // stream:handleEvent:


#pragma  mark C Funcs

//-----------------------------------------------------------------------
SCNQuaternion SCNQuaternionMultiply (SCNQuaternion q1, SCNQuaternion q2)
//-----------------------------------------------------------------------
// Multiply two quaternions. This applies rotation q2 to rotation q1.
{
    GLKQuaternion q1glk = // turn q1 into a GLKQuaternion
    GLKQuaternionMakeWithAngleAndAxis (q1.w, q1.x, q1.y, q1.z);
    q1glk = GLKQuaternionNormalize (q1glk);
    GLKQuaternion q2glk = // turn q2 into a GLKQuaternion
    GLKQuaternionMakeWithAngleAndAxis (q2.w, q2.x, q2.y, q2.z);
    q2glk = GLKQuaternionNormalize (q2glk);
    GLKQuaternion resglk = GLKQuaternionMultiply (q1glk, q2glk);
    return glk2SCN (resglk);
}

//-----------------------------------------------------------------------
SCNQuaternion SCNQuaternionInvert (SCNQuaternion q)
//-----------------------------------------------------------------------
// Find the inverse of q
{
    GLKQuaternion qglk = GLKQuaternionMakeWithAngleAndAxis (q.w, q.x, q.y, q.z);
    //qglk = GLKQuaternionNormalize (qglk);
    GLKQuaternion resglk = GLKQuaternionInvert (qglk);
    return glk2SCN (resglk);
}

//-----------------------------------------------------------------------
SCNQuaternion SCNQuaternionNorm (SCNQuaternion q)
//-----------------------------------------------------------------------
// Norm an SCNQuaternion
{
    float len = sqrt (q.x*q.x + q.y*q.y + q.z*q.z);
    q.x /= len; q.y /= len; q.z /= len;
    return q;
}

//-----------------------------------------------------------------------
SCNQuaternion glk2SCN (GLKQuaternion qglk)
//-----------------------------------------------------------------------
// Make a SCNQuaternion from a GLKQuaternion
{
    GLKVector3 axis = GLKQuaternionAxis (qglk);
    float angle = GLKQuaternionAngle (qglk);
    SCNQuaternion res = SCNVector4Make (axis.x, axis.y, axis.z, angle);
    return res;
}

//------------------------------------------
NSString *nsprintf (NSString *format, ...)
//------------------------------------------
{
    va_list args;
    va_start(args, format);
    NSString *msg =[[NSString alloc] initWithFormat:format
                                          arguments:args];
    return msg;
} // nsprintf()

//---------------------------------------------
BOOL strmatch (NSString *str, NSString *pat)
//---------------------------------------------
// Check whether string matches regex
{
    NSRegularExpression *re =
    [NSRegularExpression regularExpressionWithPattern:pat options:0 error:NULL];
    NSTextCheckingResult *match =
    [re firstMatchInString:str options:0 range:NSMakeRange(0, [str length])];
    return [match numberOfRanges]?YES:NO;
}

//-----------------------------------
float eulerPhi (SCNQuaternion p_q)
//-----------------------------------
{
    GLKQuaternion q =
    GLKQuaternionMakeWithAngleAndAxis (p_q.w, p_q.x, p_q.y, p_q.z);
    float q0 = q.w;
    float q1 = q.x;
    float q2 = q.y;
    float q3 = q.z;
    float res =
    atan2 (2.0 * (q0*q1+q2*q3), 1.0 - 2.0 * (q1*q1+q2*q2));
    return res;
}
//-----------------------------------
float eulerTheta (SCNQuaternion p_q)
//-----------------------------------
{
    GLKQuaternion q =
    GLKQuaternionMakeWithAngleAndAxis (p_q.w, p_q.x, p_q.y, p_q.z);
    float q0 = q.w;
    float q1 = q.x;
    float q2 = q.y;
    float q3 = q.z;
    float res =
    asin (2.0 * (q0*q2-q3*q1));
    return res;
}
//-----------------------------------
float eulerPsi (SCNQuaternion p_q)
//-----------------------------------
{
    GLKQuaternion q =
    GLKQuaternionMakeWithAngleAndAxis (p_q.w, p_q.x, p_q.y, p_q.z);
    float q0 = q.w;
    float q1 = q.x;
    float q2 = q.y;
    float q3 = q.z;
    float res =
    atan2 (2.0 * (q0*q3+q1*q2), 1.0 - 2.0 * (q2*q2+q3*q3));
    return res;
}

//-----------------
NSString *getIP()
//-----------------
// Get our IP address
{
    NSArray *addresses = [[NSHost currentHost] addresses];
    NSString *ip;
    for (NSString *addr in addresses) {
        if (strmatch(addr,@"[0-9]+[.][0-9]+[.][0-9]+[.][0-9]+")) {
            if (!strmatch(addr,@"127.0.0.1")) {
                ip = addr;
                break;
            }
        }
    } // for
    return ip;
}

//---------------------------------------
NSArray *splitOnNewLine (NSString *str)
//---------------------------------------
{
    NSArray *res =
    [str componentsSeparatedByCharactersInSet: [NSCharacterSet newlineCharacterSet]];
    return res;
}

//---------------------------------------
NSArray *splitOnComma (NSString *str)
//---------------------------------------
{
    NSArray *res =
    [str componentsSeparatedByString:@","];
    return res;
}

@end


