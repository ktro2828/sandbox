# Ownership
## Rules
- rustでは，**各値は"所有者"(=owner)を持つ**
- 各瞬間において所有者は**1人のみ**
- 所有者がスコープ外のとき，その値はドロップする
