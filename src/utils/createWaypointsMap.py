
import numpy as np
from matplotlib import pyplot as plt
waypoints = [(2.909995283569139, 0.6831924746239328), (3.3199952311658905, 0.6833390533713652), (3.41999521838461, 0.6833748042853732), (3.6300023417267235, 0.6834498837610459), (4.189995119968753, 0.6836500863232341), (4.500002230529587, 0.6837609167129147), (4.549995073956144, 0.6837787896136626), (5.320002125723089, 0.6840540742077795), (5.420002112941809, 0.6840898251217875), (5.7800020669292005, 0.684218528412216), (6.289747858140073, 0.6921400142174), (6.460906484698166, 0.7123063542781353), (6.5136980596947165, 0.7210294115664316), (6.704287871536597, 0.799598672280553), (6.836281775656231, 0.8817004790362547), (6.991663362669656, 1.0062653214908401), (7.1142074641408275, 1.1693225137564909), (7.165830682349035, 1.263426756737598), (7.280019741788613, 1.7628308313393968), (7.272892208655982, 1.8132370038722583), (7.265960701310593, 1.8622568749360433), (7.1045747673751585, 2.3014874894475916), (7.011749008840918, 2.419260292916218), (6.727273712845888, 2.6474924751765463), (6.536921216759571, 2.7266447610626687), (6.079802178702642, 2.773360773339069), (5.919813651266964, 2.772005974951175), (5.719827991972368, 2.7703124769663074), (5.670000926947205, 2.7698905365406308), (5.200034627604903, 2.765910816276192), (5.049876033335467, 2.7646392587170006), (5.002030872389276, 2.768980714618128), (4.942709994269048, 2.775327848322301), (4.561340171137485, 2.898322513024676), (4.258533108743229, 3.166955220685885), (4.092728535429521, 3.3703748558215287), (4.001121969780925, 3.482763638518189), (3.774000078716213, 3.761411273431655), (3.6823935130676184, 3.8738000561283137), (3.5490587458571623, 4.037383660336441), (3.2758532950668884, 4.333295323360169), (3.1911463583891155, 4.385684825652305), (3.0954945192403103, 4.435922305057415), (2.9549738926202442, 4.484413606024224), (2.8089822299540046, 4.500038654567632), (2.8110045575773057, 4.499832029419236), (2.5003276964136627, 4.498718163592657), (2.249377566090162, 4.491428972830993), (1.990177178741659, 4.483900142037221), (1.7395172672798365, 4.476619381080485), (1.1871156114665855, 4.391792930201858), (1.1054389398706574, 4.3402307341807065), (0.7316196323127645, 3.819658838269335), (0.7080468873794841, 3.5295953182618844), (0.8747319412102282, 2.7251244177375193), (0.8863119620897287, 2.6692358445815714), (0.9180990438541362, 2.5158220758940644), (0.9380374746317692, 2.4195933679559642), (1.0212099341560652, 2.0181787127447155), (1.043063552869095, 1.912706746772055), (1.0936256517149223, 1.6686792454688633), (1.219724413480236, 1.169889412099395), (1.2404620134668318, 1.1182110370035536), (1.286611404297767, 1.0270193376917442), (1.3195344250237366, 0.9895904728963364), (1.3897426105955222, 0.9097735962139227), (1.4563853812178036, 0.8435308547287804), (1.4996428710531535, 0.8193608401945228), (2.0400025449490777, 0.6828814442283201), (2.7500024542019887, 0.6831352757177762), (2.909995283569139, 0.6831924746239328)]
# labels = ['{0}'.format(i) for i in range(len(waypoints))]
x, y, lX, lY = [], [], [], []
for i, e in enumerate(waypoints):
    x = e[0]
    y = e[1]
    plt.scatter(x, y, marker='x', color='red')
    plt.text(x + 0.05, y + 0.05, str(i), fontsize=9)
    # plt.text(lX, lY, labels)

# plt.plot(x, y, 'xb-', marker='x')
# plt.text(lX, lY, labels)
plt.show()