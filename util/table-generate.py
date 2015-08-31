
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
            lens = map(int, sp[1:])
            keyinfo[key_str] = {
                'lens': lens,
                'size': reduce( lambda x,y:x*y, lens, 1 )
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
            obegin.append(0)
        else:
            obegin.append(oend[i-1])
        oend.append(obegin[i] + info['size'])
    total_size = oend[len(keys)-1]
    yield '`define type_'+name+'(x) ['+str(total_size-1)+':0] x'
    for i in range(len(keys)):
        key  = keys[i]
        info = keyinfo[key]
        lens = info['lens']
        ob   = obegin[i]
        elen = info['size']
        yield '`define ref_'+name+'_'+key+'(x)  x['+str(ob+elen-1)+':'+str(ob)+']'
        for j in range(2, 1+len(lens)):
            s_base = str(ob)
            mysize = elen
            for k in range(1, j):
                mysize /= lens[k]
                s_base += ' + a'+str(k)+'*'+str(mysize)
            s = '`define refa'+str(j-1)+'_'+name+'_'+key+'(x'
            for k in range(1, j):
                s += ' , a'+str(k)
            s += ') x['+s_base+' + '+str(mysize-1)+' : '+s_base+']'
            yield s
    
def main():
    args, keys, keyinfo = load_data(sys.stdin)
    for x in generate_iter(args, keys, keyinfo):
        sys.stdout.write(x+'\n')

    
main()

