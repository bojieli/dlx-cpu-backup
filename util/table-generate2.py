
import sys
import os

def load_data(f):
    args    = {}
    keys    = []
    keyinfo = {}
    for line in f.readlines():
        sp = line.split()
        if len(sp) == 0:
            continue
        if sp[0][0] == '#':
            continue
        key_type = sp[0][0]
        key_str  = sp[0][1:]
        if key_type == '$':
            args[key_str] = sp[1]
        elif key_type == '.':
            keys.append(key_str)
            width = sp[1]
            sizes = sp[2:]
            sizes.append(width)
            lmuls = map( lambda l: '*'.join(l) if len(l) > 0 else 1, [ sizes[:i] for i in range(len(sizes)) ] )
            rmuls = map( lambda l: '*'.join(l) if len(l) > 0 else 1, [ sizes[i:] for i in range(len(sizes)) ] )
            keyinfo[key_str] = {
                'width': width,
                'lmuls': lmuls,
                'rmuls': rmuls,
                'size': rmuls[0]
            }
        else:
            raise ValueError
    return args, keys, keyinfo

def generate_iter(args, keys, keyinfo):
    name = args['name']
    obegin = []
    oend   = []
    for i in range(len(keys)):
        key  = keys[i]
        info = keyinfo[key]
        if i == 0:
            obegin.append('0')
        else:
            obegin.append(oend[i-1])
        oend.append(obegin[i] + '+' + info['size'])
    total_size = oend[len(keys)-1]
    yield '`define width_'+name+' '+total_size
    yield '`define type_'+name+'(x) ['+total_size+'-1:0] x'
    yield '`define form_'+name+' ['+total_size+'-1:0]'
    for i in range(len(keys)):
        key   = keys[i]
        keyname = '_'+key if len(key)>0 else ''
        info  = keyinfo[key]
        width = info['width']
        lmuls = info['lmuls']
        rmuls = info['rmuls']
        yield '`define ref_'+name+keyname+'(x)  x['+oend[i]+'-1:'+obegin[i]+']'
        for j in range(1, len(lmuls)):
            s_base = obegin[i]
            for k in range(1, j+1):
                s_base += ' +'+rmuls[k] +'*(a'+str(k)+')'
            s = '`define refa'+str(j)+'_'+name+keyname+'(x'
            for k in range(1, j+1):
                s += ' , a'+str(k)
            s += ') x['+s_base+' +'+rmuls[j]+'-1: '+s_base+']'
            yield s
        if len(lmuls) == 1:
            yield '`define '+name+keyname+'(x) `ref_'+name+keyname+'(x)'
        else:
            yield '`define '+name+keyname+'(x,y) `refa'+str(len(lmuls)-1)+'_'+name+keyname+'(x,y)'
    
def main():
    args, keys, keyinfo = load_data(sys.stdin)
    for x in generate_iter(args, keys, keyinfo):
        sys.stdout.write(x+'\n')

    
main()

