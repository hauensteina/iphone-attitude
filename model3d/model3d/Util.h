//
//  Util.h
//  model3d
//
//  Created by Andreas Hauenstein on 2015-03-04.
//  Copyright (c) 2015 AHN. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface Util : NSObject
+ (NSArray *) readLinesFromFile:(NSString*)fname;
+ (NSString *) replaceRegex:(NSString *)theRegex
                   inString:(NSString *)source
                 withString:(NSString *)newStr;


BOOL nstrstr (NSString *haystack, NSString *needle);
NSString *trim (NSString *str);
@end
