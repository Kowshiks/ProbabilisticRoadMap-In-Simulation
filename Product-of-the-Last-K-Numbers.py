class ProductOfNumbers:

    def __init__(self):
        
        self.arr = []
        self.cache = [1]
        self.index = 0
        
    def add(self, num: int) -> None:
        
        if num == 0:
            
            self.cache = [1]
            self.index = 0
            
        else:
        
            self.arr.append(num)
            self.cache.append(self.cache[self.index]*num)
            self.index+=1


    def getProduct(self, k: int) -> int:
    
        
        if k > len(self.cache) - 1:
                                
            return 0
        
        point = len(self.cache) - k - 1
        
        return int(self.cache[-1]/self.cache[point])
        


# Your ProductOfNumbers object will be instantiated and called as such:
# obj = ProductOfNumbers()
# obj.add(num)
# param_2 = obj.getProduct(k)
