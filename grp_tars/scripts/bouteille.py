class Bouteille:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.samplex = []
        self.sampley = []
        self.placed = False
        self.needToBeMarked = False

    def sameAs(self, bouteille, margin):
        return abs(self.x - bouteille.x) < margin and abs(self.y - bouteille.y) < margin

    def alreadyIn(self, bouteilles, margin):
        flag = False
        for bouteille in bouteilles:
            if self.sameAs(bouteille, margin):
                flag = True
                break
        return flag

    def numberSeen(self):
        return len(self.samplex)

    def meanPosition(self):
        return sum(self.samplex) / len(self.samplex), sum(self.sampley) / len(self.sampley)

    def getPosition(self):
        if self.placed:
            x = self.x
            y = self.y
        else:
            x, y = self.meanPosition()
        return x, y

    def addtoSample(self, x, y):
        self.samplex.append(x)
        self.sampley.append(y)

    def placeBottle(self):
        self.x, self.y = self.meanPosition()
        self.placed = True
        self.needToBeMarked = True
