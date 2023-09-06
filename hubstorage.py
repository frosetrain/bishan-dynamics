from pybricks.hubs import PrimeHub
from ustruct import calcsize, pack_into, unpack_from

hub = PrimeHub()

# buf = bytearray(8 * 2)  # 2 bytes for each unsigned short
# ref = [123, 453, 234, 665, 321, 234, 543, 233]
# i = 0
# for d in ref:
#     pack_into("H", buf, i, d)
#     i += 2
# hub.system.storage(0, write=buf)

dump = hub.system.storage(0, read=16*2)
out = []
for i in range(16):
    out.append(unpack_from("H", dump, i * 2)[0])
print(out)
