class Bouteille:
    def __init__(self, id='b0', x=0, y=0):
        self.id = id
        self.x = x
        self.y = y

    def sameAs(self, bouteille, margin):
        return abs(self.x - bouteille.x) < margin and abs(self.y - bouteille.y) < margin

    def alreadyIn(self, bouteilles, margin):
        flag = False
        for bouteille in bouteilles:
            if self.sameAs(bouteille, margin):
                flag = True
                break
        return flag
