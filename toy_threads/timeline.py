import blessed


class Timeline:
    def __init__(self, fields, colors=['blue', 'green', 'red', 'yellow']):
        self.term = blessed.Terminal()
        self.fields = fields
        self.data = {}
        self.colors = {}
        self.max_width = 0
        self.t0 = 0.0
        self.max_t = 0.0
        self.resolution = 0.05

        for i, field in enumerate(fields):
            if isinstance(colors, str):
                cs = colors
            else:
                cs = colors[i % len(colors)]
            self.colors[field] = getattr(self.term, cs)
            self.data[field] = []
            self.max_width = max(self.max_width, len(field))
            print()
        self.template = '{field:' + str(self.max_width + 1) + '} {freq:5.2f} Hz | '

    def start(self, label, ts):
        self.data[label].append((ts, None))
        self.max_t = max(self.max_t, ts + self.resolution)
        self.draw()

    def end(self, label, ts):
        start, d = self.data[label][-1]
        self.data[label][-1] = start, ts - start
        self.max_t = max(self.max_t, ts)
        self.draw()

    def draw(self):
        h = self.term.height
        N = self.term.width - 30

        max_t = self.t0 + N * self.resolution
        if self.max_t > max_t:
            self.t0 += self.resolution * N / 2
            max_t = self.t0 + N * self.resolution

        ts = [self.t0 + i * self.resolution for i in range(N)]
        for i, field in enumerate(self.fields):
            y = h - (len(self.fields) - i) - 1
            freq = len(self.data[field]) / self.max_t
            s = self.template.format(**locals())

            data_i = 0
            while data_i < len(self.data[field]):
                start, d = self.data[field][data_i]
                if d is None:
                    break
                elif self.t0 > start + d:
                    data_i += 1
                else:
                    break

            for t in ts:
                if data_i >= len(self.data[field]):
                    if t <= self.max_t:
                        s += '-'
                    else:
                        s += ' '
                    continue
                start, d = self.data[field][data_i]
                if t < start + self.resolution:
                    s += '-'
                elif d is None:
                    if t <= self.max_t:
                        s += 'x'
                    else:
                        s += ' '
                elif t < start + d:
                    s += 'x'
                else:
                    data_i += 1
                    s += 'x'

            print(self.term.move_xy(0, y) + self.colors[field] + s + self.term.normal)
