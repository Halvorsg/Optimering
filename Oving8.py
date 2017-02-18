stdin=open("input01.txt","r")
#fucker opp ved 98
#from sys import stdin

Inf = 1000000000


def min_coins_greedy(coins, value):
    antall = 0
    for c in coins:
        while value >= c:
            value -= c
            antall += 1
    return antall

def min_coins_dynamic(coins, value):
    value = 2567
    x =[0]*(value+1)
    for i in range(0,value+1):
        x[i] = i
        if i in coins:
            x[i] = 1
    coins.reverse()
    for i in range(1,value+1):
        for coin in coins:
            #print(coin)
            if (i+coin)<=value:
                if x[i+coin] >= x[i]+1: x[i+coin] = x[i]+1
    
    return x[value]
    
    #return matrix[value]
    



def get_matrix(coins, value):
    if len(coins)==1:
        return value
    if value == 0:
        return value
    x =[0]*(value+1)
    for i in range(0,value+1):
        x[i] = i
        if i in coins:
            x[i] = 1
    coins.reverse()
    for i in range(1,value+1):
        for coin in coins:
            #print(coin)
            if (i+coin)<=value:
                if x[i+coin] >= x[i]+1: x[i+coin] = x[i]+1
    
    return x


def can_use_greedy(coins):
    return False

coins = []
for c in stdin.readline().split():
    coins.append(int(c))
coins.sort()
coins.reverse()
method = stdin.readline().strip()
if method == "graadig" or (method == "velg" and can_use_greedy(coins)):
    for line in stdin:
        print(min_coins_greedy(coins, int(line)))
else:
    #x =[]
    #for line in stdin:
    #    x.append(int(line))
    #matrix = get_matrix(coins,max(x))
    for line in stdin:
        print(min_coins_dynamic(coins, int(line)))
        
        
        
        