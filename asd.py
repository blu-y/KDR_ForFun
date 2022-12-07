import sys
import string
import re #정규 표현식
import struct

cond = {'eq': 0, 'ne': 1, 'hs': 2, 'cs': 2, 'lo': 3,
        'cc': 3, 'mi': 4, 'pl': 5, 'vs': 6, 'vc': 7,
        'hi': 8, 'ls': 9, 'ge': 10, 'lt': 11, 'gt': 12,
        'le': 13, 'al': 14, 'nv': 15}

registers = {'r0': 0, 'r1': 1, 'r2': 2, 'r3': 3, 'r4': 4,
             'r5': 5, 'r6': 6, 'r7': 7, 'r8': 8, 'r9': 9,
             'r10': 10, 'r11': 11, 'r12': 12, 'r13': 13,
             'r14': 14, 'r15': 15, 'sl': 10, 'fp': 11,
             'ip': 12, 'sp': 13, 'lr': 14, 'pc': 15}

def make_regexp(li):
    res = '('
    for elem in li:
        res += elem + '|'
    res = string.rstrip(res, '|')
    res += ')'
    return res

cond_regexp = make_regexp(cond.keys())

def process_cond_field(mach_code, tok):
    cond_field = tok[:2]
    if cond_field in cond:
        mach_code |= cond[cond_field] << 28
        tok = tok[2:]
#        print('\tCOND is set to ' + str(cond[cond_field]))
        print('\tCOND is set to ' + str(bin(cond[cond_field])))
    else: # if cond is undefined
        mach_code |= 14 << 28
        print('\tCOND is undefined')
    return (mach_code, tok)

def process_S_flag(mach_code, tok):
    if tok == 's':
        print('\tS flag is set')
        mach_code |= 1 << 20
        tok = tok[1:]
    return (mach_code, tok)

def process_2_args(mach_code, args):
    # match_reg is list of matching register
    if args[0] in registers:
        mach_code |= registers[args[0]] << 12
    else: # destination must be register
        print('ERROR: Invalid operand')
        sys.exit(1)

    if args[1] != ',':
        print('ERROR: Invalid syntax')
        sys.exit(1)

    if args[2] in registers:
        mach_code |= registers[args[2]]
    elif args[2][0] == '#':
        pass
    else: # operand is neither register nor constant
        print('ERROR: Invalid operand')
        sys.exit(1)

    return mach_code

def process_instruction(tokens):
    mach_code = 0
    tok = tokens[0]
    args = tokens[1:]

    # mov family
    mov_re = 'mov' + cond_regexp + '?' + 's' + '?'
    print('\treg exp for mov: ' + mov_re)
    if re.match(mov_re, tok):
        print('\tMOV FAMILY')
        mach_code = 0b1101 << 21
        tok = tok[3:]

        (mach_code, tok) = process_cond_field(mach_code, tok)
        (mach_code, tok) = process_S_flag(mach_code, tok)
        mach_code = process_2_args(mach_code, args)
        
    return mach_code

### main() starts here ###
    
lines = sys.stdin.readlines()
splitter = re.compile(r'([ \t\n,])')

for line in lines:
    tokens = splitter.split(line)
    print(tokens)
    tokens = [tok for tok in tokens
              if re.match('\s*$', tok) == None]
    print(tokens)

    mach_code = 0
    while len(tokens) > 0:
        if tokens[0].endswith(':'): # process label
            print('\tLABEL ' + tokens[0].rstrip(':') + ' FOUND')
            tokens = tokens[1:]
            continue
        elif tokens[0].startswith('.'): # process directive
            print('\tDIRECTIVE ' + tokens[0] + ' FOUND')
            tokens = tokens[1:]
            continue
        else: # process instruction
            mach_code = process_instruction(tokens)
            print(hex(mach_code) + ' : Machine Instruction\n')
            print (struct.pack('I', mach_code))
            break
