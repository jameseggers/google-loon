class LoonBalloons:
    def __init__(self):
        self.paramsFile = open('loon.in', 'r').readlines()
        self.params = {}
        self.balloons = []
        self.startOfWindVectors = 1054

    def process(self):
        self.storeRCA()
        self.storeLVBT()
        self.storeStartingCell()
        self.storeTargetCell()
        self.storeWindVectors()
        self.storeBalloons()
        self.params['balloons'] = self.balloons
        return self.params

    def storeBalloons(self):
        for balloon_id in range(1, self.params['B']):
            self.balloons.append(
            {
                "position": self.params['startingCell'],
                "altitude": 0,
                "balloon_id": balloon_id
            })

    def storeRCA(self):
        fLineSplited = self.paramsFile[0].split(' ')
        self.params['R'] = int(fLineSplited[0])
        self.params['C'] = int(fLineSplited[1])
        self.params['A'] = int(fLineSplited[2])

    def storeLVBT(self):
        sLineSplited = self.paramsFile[1].split(' ')
        self.params['L'] = int(sLineSplited[0])
        self.params['V'] = int(sLineSplited[1])
        self.params['B'] = int(sLineSplited[2])
        self.params['T'] = int(sLineSplited[3])

    def storeStartingCell(self):
        tLineSplited = self.paramsFile[2].split(' ')
        self.params['startingCell'] = (int(tLineSplited[0]), int(tLineSplited[1]))

    def storeTargetCell(self):
        self.params['targetCells'] = []
        counter = 3

        while counter <= self.params['L']:
            splited = self.paramsFile[counter].split(' ')
            self.params['targetCells'].append((int(splited[0]), int(splited[1])))
            counter += 1

    def storeWindVectors(self):
        self.params['windVectors'] = [  ]
        positionOfWindVectors = 3 + self.params['L']
        counter = 0
        currentAltitude = 0

        for altitude in range(1, self.params['A']):
            if len(self.params['windVectors']) < altitude:
                self.params['windVectors'].append([])

            for row in range(0, self.params['R']):
                if len(self.params['windVectors'][altitude-1]) < (row+1):
                    self.params['windVectors'][altitude-1].append([])

                lineNum = (self.startOfWindVectors + row) if altitude is 0 else (self.startOfWindVectors + (row * altitude))
                splitedLine = self.paramsFile[lineNum].split(' ')
                vectors = [splitedLine[n:n+2] for n in range(0, len(splitedLine), 2)]

                self.params['windVectors'][altitude-1][row].append(vectors)

LoonBalloons().process()
