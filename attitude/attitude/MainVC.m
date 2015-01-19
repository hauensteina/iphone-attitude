//
//  MainVC.m
//  attitude
//
//  Created by Andreas Hauenstein on 2015-01-13.
//  Copyright (c) 2015 AHN. All rights reserved.
//

#import "MainVC.h"
#import <CoreMotion/CMMotionManager.h>
#import <math.h>

// Streams for socket communication with server
CFReadStreamRef mReadStream = nil;
CFWriteStreamRef mWriteStream = nil;
NSInputStream *mIStream = nil;
NSOutputStream *mOStream = nil;

@interface MainVC ()

// IB Stuff
@property (weak, nonatomic) IBOutlet UILabel *lbx;
@property (weak, nonatomic) IBOutlet UILabel *lby;
@property (weak, nonatomic) IBOutlet UILabel *lbz;
@property (weak, nonatomic) IBOutlet UILabel *lbAngle;

@property (weak, nonatomic) IBOutlet UITextField *txtIPA;
@property (weak, nonatomic) IBOutlet UITextField *txtIPB;
@property (weak, nonatomic) IBOutlet UITextField *txtIPC;
@property (weak, nonatomic) IBOutlet UITextField *txtIPD;
@property (weak, nonatomic) IBOutlet UITextField *txtPort;

@property (weak, nonatomic) IBOutlet UIButton *btnLED;


// Motion stuff
@property NSOperationQueue *deviceQueue;
@property CMMotionManager *motionManager;
@property float x;
@property float y;
@property float z;
@property float angle;

// Other
@property NSTimer *connectTimeout;
@property BOOL isConnected;

@end

@implementation MainVC

#pragma  mark View LifeCycle

//---------------------
- (void)viewDidLoad
//---------------------
{
    [super viewDidLoad];
    _lbx.text = @"0";
    _lby.text = @"0";
    _lbz.text = @"0";
    _lbAngle.text = @"0";
    self.view.frame = [[UIScreen mainScreen] bounds];
    
    [_txtIPA setDelegate:self];
    [_txtIPB setDelegate:self];
    [_txtIPC setDelegate:self];
    [_txtIPD setDelegate:self];
    [_txtPort setDelegate:self];
    [self initIP];
    
    // Motion manager
    _deviceQueue = [[NSOperationQueue alloc] init];
    _motionManager = [CMMotionManager new];
    if (_motionManager.isDeviceMotionAvailable) {
        _motionManager.deviceMotionUpdateInterval = 0.1;
        [self.motionManager
         //startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryCorrectedZVertical
         startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
         toQueue:self.deviceQueue
         withHandler:^(CMDeviceMotion *m, NSError *error)
         {
             [[NSOperationQueue mainQueue] addOperationWithBlock:^{
                 CMQuaternion q = m.attitude.quaternion;
                 float theta_rad = 2.0 * acos(q.w);
                 float theta_deg = theta_rad * (180.0 / M_PI);
                 float sin_theta_2 = sin (theta_rad / 2.0);
                 float x = q.x / sin_theta_2;
                 float y = q.y / sin_theta_2;
                 float z = q.z / sin_theta_2;
                 // Norm the vector to 1.0
                 //float vlen = sqrt (x*x+y*y+z*z);
                 //x /= vlen; y /= vlen; z /= vlen;
                 
                 _lbx.text = nsprintf(@"%.2f", x);
                 _lby.text = nsprintf(@"%.2f", y);
                 _lbz.text = nsprintf(@"%.2f", z);
                 _lbAngle.text = nsprintf (@"%.2f", theta_deg);
                 
                 if (_isConnected) {
                     NSString *msg = nsprintf (@"%.4f,%.4f,%.4f,%.4f\n",theta_rad,x,y,z);
                     [self write2server:msg];
                 }
            }];
         }];
    }
} // viewDidLoad()


#pragma mark Button Callbacks

//-------------------
- (void) cbConnect
//-------------------
{
    [self disconnectServer];

    if ([_txtIPA.text length] == 0
        || [_txtIPB.text length] == 0
        || [_txtIPC.text length] == 0
        || [_txtIPD.text length] == 0)
    {
        [self popup:@"IP part missing" title:@"Error"]; return;
    }
    if ([_txtPort.text length] == 0) {
        [self popup:@"Port missing" title:@"Error"]; return;
    }
    int ipa = [_txtIPA.text intValue];
    int ipb = [_txtIPB.text intValue];
    int ipc = [_txtIPC.text intValue];
    int ipd = [_txtIPD.text intValue];
    int port = [_txtPort.text intValue];
    if (ipa > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (ipb > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (ipc > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (ipd > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (port > 0xffff) { [self popup:@"illegal Port" title:@"Error"]; return; }
    
    [self putNum:@"IPA" val:@(ipa)];
    [self putNum:@"IPB" val:@(ipb)];
    [self putNum:@"IPC" val:@(ipc)];
    [self putNum:@"IPD" val:@(ipd)];
    [self putNum:@"PORT" val:@(port)];
    
    NSString *server = nsprintf (@"%d.%d.%d.%d",ipa,ipb,ipc,ipd);
    [self connect2server:server port:port];
        //[self write2server:@"start_stream"];
    [self ledAmber];
} // cbConnect()

//----------------------------------
- (IBAction)btnConnect:(id)sender
//----------------------------------
{
    if (!_isConnected) {
        [self cbConnect];
    } else {
        [self disconnectServer];
    }
}

//-----------------------------
- (IBAction)btnLed:(id)sender
//-----------------------------
{
    [self btnConnect:sender];
}


#pragma mark IP Address

//----------------------------------------------------------
- (BOOL)               textField:(UITextField *)textField
   shouldChangeCharactersInRange:(NSRange)range
               replacementString:(NSString *)str
//----------------------------------------------------------
// Restrict all textfield input to digits only
{
    if (!strmatch(str,@"^[0-9]*$")) { return NO; }
    int maxlen = 3;
    if (textField == _txtPort) { maxlen = 5; }
    long len = [textField.text length] + [str length] - range.length;
    return (len > maxlen) ? NO : YES;
}

//-----------------
- (void) initIP
//-----------------
// Init textfields to values stored in userdefaults
{
    _txtIPA.text = [self getStr:@"IPA"];
    _txtIPB.text = [self getStr:@"IPB"];
    _txtIPC.text = [self getStr:@"IPC"];
    _txtIPD.text = [self getStr:@"IPD"];
    _txtPort.text = [self getStr:@"PORT"];
}

#pragma  mark Server Connectivity

//-------------------------------------------------------------
- (void)connectTimeout:(NSTimer*)timer
//-------------------------------------------------------------
// Socket connection timeout
{
    [self disconnectServer];
    [self popup:@"Connect failed" title:@"Error"];
}


// Make a TCP/IP connection to a server.
// Sets the member variables mIStream and mOStream.
// Use them to communicatte with the server.
//----------------------------------------
- (BOOL)connect2server:(NSString *)server
                  port:(int)port
//----------------------------------------
{
    
    // Give up after some seconds
    _connectTimeout =
    [NSTimer scheduledTimerWithTimeInterval: 4.0
                                     target: self
                                   selector: @selector(connectTimeout:)
                                   userInfo: nil
                                    repeats: NO];
    
    CFStreamCreatePairWithSocketToHost(kCFAllocatorDefault,
                                       (__bridge CFStringRef) server,
                                       port,
                                       &mReadStream,
                                       &mWriteStream);
    
    if (mReadStream && mWriteStream) {
        CFReadStreamSetProperty (mReadStream,
                                kCFStreamPropertyShouldCloseNativeSocket,
                                kCFBooleanTrue);
        CFWriteStreamSetProperty (mWriteStream,
                                 kCFStreamPropertyShouldCloseNativeSocket,
                                 kCFBooleanTrue);
        
        mIStream = (__bridge NSInputStream *)mReadStream;
        //[mIStream retain];
        [mIStream setDelegate:self];
        [mIStream scheduleInRunLoop:[NSRunLoop mainRunLoop]
                            forMode:NSDefaultRunLoopMode];
        [mIStream open];
        
        mOStream = (__bridge NSOutputStream *)mWriteStream;
        //[mOStream retain];
        [mOStream setDelegate:self];
        [mOStream scheduleInRunLoop:[NSRunLoop mainRunLoop]
                            forMode:NSDefaultRunLoopMode];
        [mOStream open];
        
        return YES;
    }
    return NO;
} // connect2server()

//--------------------------
- (void) ledGreen
//--------------------------
{
    UIImage *greenLED = [UIImage imageNamed:@"green-led-on-md.png"];
    [_btnLED setImage:greenLED forState:UIControlStateNormal];
    _isConnected = YES;
}

//--------------------------
- (void) ledRed
//--------------------------
{
    UIImage *redLED = [UIImage imageNamed:@"led-red-control-md.png"];
    [_btnLED setImage:redLED forState:UIControlStateNormal];
    _isConnected = NO;
}

//--------------------------
- (void) ledAmber
//--------------------------
{
    UIImage *amberLED = [UIImage imageNamed:@"amber-led-on-md.png"];
    [_btnLED setImage:amberLED forState:UIControlStateNormal];
    _isConnected = NO;
}

// Disconnect from server
//--------------------------
- (void)disconnectServer
//--------------------------
{
    if (mIStream != nil) [mIStream close];
    if (mOStream != nil) [mOStream close];
    mIStream = nil;
    mOStream = nil;
    [self ledRed];
} // disconnectServer()

// Write a null terminated string to the server
//-------------------------------------------
-(void) write2server:(NSString *)p_msg
//-------------------------------------------
{
    const char *msg = [p_msg cStringUsingEncoding:NSUTF8StringEncoding];
    [mOStream write:(uint8_t *)msg maxLength:strlen(msg)];
} // write2server()

// Stream events and data from the server
//------------------------------------------------------------------------
- (void)stream:(NSStream *)stream handleEvent:(NSStreamEvent)eventCode
//------------------------------------------------------------------------
{
    if (_connectTimeout) {
        [_connectTimeout invalidate];
        _connectTimeout = nil;
    }
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
            if ([str length]) {
                [self popup:str title:@"Server Message"];
            }
            break;
        }
        case NSStreamEventErrorOccurred: {
            [self disconnectServer];
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
            [self ledGreen];
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
} // handleEvent

#pragma mark UI Helpers

//-------------------------------
- (void) popup:(NSString *)msg
         title:(NSString *)title
//-------------------------------
{
    UIAlertView *alert =
    [[UIAlertView alloc] initWithTitle:title
                               message:msg
                              delegate:self
                     cancelButtonTitle:@"OK"
                     otherButtonTitles:nil];
    [alert show];
}

#pragma mark Userdefaults

#define DEF [NSUserDefaults standardUserDefaults]

//-----------------------------------------------------
- (void) putNum:(NSString *)key val:(NSNumber *)val
//-----------------------------------------------------
// Store a number in UserDefaults
{
    [DEF setObject:val forKey:key];
}

//-----------------------------------------------------
- (NSNumber *) getNum:(NSString *)key
//-----------------------------------------------------
// Get number from UserDefaults
{
    return [DEF objectForKey:key];
}

//-----------------------------------------------------
- (int) getInt:(NSString *)key
//-----------------------------------------------------
// Get number from UserDefaults, return as int
{
    return [[DEF objectForKey:key] intValue];
}

//-----------------------------------------------------
- (NSString *) getStr:(NSString *)key
//-----------------------------------------------------
// Get object from UserDefaults, return as string
{
    id obj = [DEF objectForKey:key];
    return obj ? nsprintf (@"%@", [DEF objectForKey:key]) : @"" ;
}

#pragma  mark C Funcs

//----------------------------
NSString *nsprintf (NSString *format, ...)
//----------------------------
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

#pragma Cruft

//--------------------------------------------------------
-(BOOL) textFieldShouldReturn:(UITextField *)textField
//--------------------------------------------------------
// Hide keyboards on return
{
    [textField resignFirstResponder];
    return YES;
}


@end



