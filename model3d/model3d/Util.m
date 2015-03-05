//
//  Util.m
//  model3d
//
//  Created by Andreas Hauenstein on 2015-03-04.
//  Copyright (c) 2015 AHN. All rights reserved.
//

#import "Util.h"

@implementation Util

//-------------------------------------------------
+ (NSArray *) readLinesFromFile:(NSString*)fname
//-------------------------------------------------
// Returns an array of NSString with one element per line.
{
    FILE *fp = fopen([fname cStringUsingEncoding:NSUTF8StringEncoding],"r");
    if (!fp) {
        NSLog(@"Failed to open file %@",fname);
        return nil;
    }
    
    char *line = NULL;
    size_t buflen = 0;
    long bytes_read;
    NSMutableArray *res = [NSMutableArray new];
    while ((bytes_read = getline (&line, &buflen, fp)) > 0) {
        NSString *linestr = [NSString stringWithCString:line
                                               encoding:NSUTF8StringEncoding];
        [res addObject:linestr];
    } // while
    fclose (fp);
    free (line);
    return res;
} // readLinesFromFile()

//-----------------------------------------
+ (NSString *) replaceRegex:(NSString *)theRegex
                   inString:(NSString *)source
                 withString:(NSString *)newStr
//-----------------------------------------
{
    NSRegularExpression *regex = [NSRegularExpression
                                  regularExpressionWithPattern:theRegex
                                  options:0
                                  error:nil];
    NSString *res = [regex stringByReplacingMatchesInString:source
                                                    options:0
                                                      range:NSMakeRange(0, [source length])
                                               withTemplate:newStr];
    return res;
} // replaceRegex



//-----------------------------
BOOL nstrstr (NSString *haystack, NSString *needle)
//-----------------------------
// Case insensitive check whether a string contains another
{
    int loc = (int)[haystack rangeOfString:needle options:NSCaseInsensitiveSearch].location;
    if (loc == (int)NSNotFound) { return NO; }
    else { return YES; }
} // nstrstr()

//----------------------------
NSString *trim (NSString *str)
//----------------------------
{
    NSString *res =
    [str stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
    return res;
}

@end
