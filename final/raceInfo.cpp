#include <cmath>
#include "raceInfo.hpp"

// 元のまま。自分のAIでは使っていない
void addSquares(int x, int y0, int y1, list <Position> &squares) {
  if (y1 > y0) {
    for (int y = y0; y <= y1; y++) {
      squares.emplace_back(x, y);
    }
  } else {
    for (int y = y0; y >= y1; y--) {
      squares.emplace_back(x, y);
    }
  }
}

// 元のまま。自分のAIでは使っていない
list <Position> Movement::touchedSquares() const {
	list <Position> r;
	int dx = to.x - from.x;
	int dy = to.y - from.y;
	int sgnx = dx > 0 ? 1 : -1;
	int sgny = dy > 0 ? 1 : -1;
	int adx = abs(dx);
	int ady = abs(dy);
	if (dx == 0) {
		for (int k = 0, y = from.y; k <= ady; k++, y += sgny) {
			r.emplace_back(from.x, y);
		}
	}
	else if (dy == 0) {
		for (int k = 0, x = from.x; k <= adx; k++, x += sgnx) {
			r.emplace_back(x, from.y);
		}
	}
	else {
		// Let us transform the move line so that it goes up and to the right,
		// that is, with dx > 0 and dy > 0.
		// The results will be adjusted afterwards.
		if (sgnx < 0) dx = -dx;
		if (sgny < 0) dy = -dy;
		// We will use the coordinate system in which the start point
		// of the move is at (0,0) and x-coodinate values are doubled,
		// so that x is integral on square borders.
		// The point (X,Y) in the original coordinate system becomes
		//   x = 2*(X-from.x)
		//   y = Y-from.y
		// Such a movement line satisfies the following.
		//   y = dy/dx/2 * x, or 2*dx*y = dy*x
		//
		// The start square and those directly above it
		for (int y = 0; dx*(2 * y - 1) <= dy; y++) {
			r.emplace_back(0, y);
		}
		// The remaining squares except for those below (dx, dy)
		for (int x = 1; x < 2 * dx - 1; x += 2) {
			for (int y = (dy*x + dx) / (2 * dx) -
				(dy*x + dx == (dy*x + dx) / (2 * dx)*(2 * dx) ? 1 : 0);
				dx*(2 * y - 1) <= dy * (x + 2);
				y++) {
				r.emplace_back((x + 1) / 2, y);
			}
		}
		// For the final squares with x = dx
		for (int y = (dy*(2 * dx - 1) + dx) / (2 * dx) -
			((dy*(2 * dx - 1) + dx) == (dy*(2 * dx - 1) + dx) / (2 * dx)*(2 * dx) ? 1 : 0);
			y <= dy;
			y++) {
			r.emplace_back(dx, y);
		}
		// Adjustment
		for (auto &p : r) {
			if (sgnx < 0) p.x = -p.x;
			if (sgny < 0) p.y = -p.y;
			p.x += from.x;
			p.y += from.y;
		}
	}
	return r;
}

// 元のまま。そのまま使用している
istream &operator>>(istream &in, RaceCourse &course) {
	in >> course.thinkTime
		>> course.stepLimit
		>> course.width >> course.length
		>> course.vision;
	return in;
}

// 元のまま。そのまま使用している
istream &operator>>(istream &in, PlayerState &ps) {
	in >> ps.position.x
		>> ps.position.y
		>> ps.velocity.x
		>> ps.velocity.y;
	return in;
}

// 基本元のままだが、障害物をビットボードに記録する部分を追加している
istream &operator>>(istream &in, RaceInfo &ri) {
	in >> ri.stepNumber;
	if (ri.stepNumber < 0) {
		return in;
	}
	in >> ri.timeLeft
		>> ri.me[0]
		>> ri.opponent[0];
	// 敵味方共に、前のターンと同じ位置、速度であるかどうかを調べる
	if (ri.stepNumber > 0 && ri.me[0].position == ri.prevpl.position && ri.me[0].velocity == ri.prevpl.velocity &&
		ri.opponent[0].position == ri.prevene.position && ri.opponent[0].velocity == ri.prevene.velocity) {
		ri.isprevsame = true;
	}
	else {
		ri.isprevsame = false;
	}
	// 障害物のビットボードをクリア
	for (int i = 0; i < BB_HEIGHT; i++) {
		ri.obst[i].clear();
	}
	// 読み込んだデータをデータに反映する
	for (int y = 0; y != ri.length; y++) {
		for (int x = 0; x != ri.width; x++) {
			int state;
			in >> state;
			ri.squares[y][x] = static_cast<char>(state);
			// 障害物だった場合は、障害物のビットボードを設定
			if (state == SquareInfo::Obstacle) {
				// ビットボード1つの高さの数だけ、y座標を一つずつずらしたものを設定する
				for (int i = 0; i < BB_HEIGHT; i++) {
					ri.obst[i].set(x, y + i);
				}
			}
		}
	}
	// 視界外の場所が存在する場合、視界外付近の地形が、あらかじめ用意されたパターンの地形と等しいかどうかをチェックし、
	// 等しければ視界外の部分をそのパターンに置き換える
	if (ri.checkpattern && ri.squares[ri.length - 1][0] == SquareInfo::Unknown) {
		// 視界内の最大のy座標の計算
		ri.sikaiy = ri.length - 1;
		while (ri.sikaiy > 0) {
			if (ri.squares[ri.sikaiy][0] != SquareInfo::Unknown) {
				break;
			}
			ri.sikaiy--;
		}
		// 次に、平原のみの行を探し、その行からあらかじめ用意されたパターンと一致するかどうかを調べる。
		// ただし、パターンの中には途中の行に平原のみを含む行が存在するので、平原のみの行に対してパターンと
		// 一致するものが見つからなかった場合は次の平原の行を探してチェックを続ける。
		// ただし、パターンの最大高さは20なので、それ以上は調べない
		// y はチェック中の平原のみの行を表す変数。
		// 初期値としてゴールの手前の行を設定しておく
		int y = ri.length - 1;
		// 視界から20マス以内で、
		// 最もゴールに近い、すべてが平地の行を探す
		// ただし、0行目は除く
		while (ri.sikaiy + 1 < y + 20) {
			// y が 0 以上の間、すべてが平原の行を探す
			while (y > 0) {
				bool isallplain = true;
				for (int x = 0; x < ri.width; x++) {
					if (ri.squares[y][x] != SquareInfo::Plain) {
						isallplain = false;
						break;
					}
				}
				if (isallplain) {
					break;
				}
				y--;
			}
			ri.patfound = false;
			// 見つかった場合で、その次の行が視界外でない場合、特定のパターンと一致しているかどうかを調べ、
			// 一致していた場合は、ri.squares をそのパターンで埋める
			if (y > 0 && y + 1 < ri.length && ri.squares[y + 1][0] != SquareInfo::Unknown) {
				//cerr << "pl line " << y << endl;
				// マップパターンに一致するものを探す。
				// ただし、最後の行まで一致していた場合は、全て見えていて意味がないのと、トラップの場合は左右反転するので最後の行はチェックしない。
				int num = 0;
				for (auto& mp : ri.mappatterns[ri.width]) {
					bool found = true;
					num++;
					if (y + mp.length - 1 >= ri.length || ri.squares[y + mp.length - 1][0] != SquareInfo::Unknown) {
						continue;
					}
					for (int y2 = 1; y2 < mp.length - 1; y2++) {
						// 見えていない行まで一致している場合
						if (ri.squares[y + y2][0] == SquareInfo::Unknown) {
							break;
						}
						for (int x = 0; x < mp.width; x++) {
							if (ri.squares[y + y2][x] != mp.squares[y2][x]) {
								found = false;
								break;
							}
						}
						if (found == false) {
							break;
						}
					}
					// checktrap が false の場合は trap のパターンは無視する
					if (found && !(ri.checktrap == false && mp.type == MAPPATTERN::TRAP)) {
						cerr << "pf " << y << " " << num << " t " << mp.type << endl;
						ri.patfound = true;
						// 見つかったパターンと位置を記録する
						ri.pat = mp;
						ri.paty = y;
						ri.sikaiy = y + mp.length - 1;
						for (int y2 = 1; y2 < mp.length; y2++) {
							for (int x = 0; x < mp.width; x++) {
								ri.squares[y + y2][x] = mp.squares[y2][x];
								// 障害物だった場合は、障害物のビットボードを設定
								if (ri.squares[y + y2][x] == SquareInfo::Obstacle) {
									// ビットボード1つの高さの数だけ、y座標を一つずつずらしたものを設定する
									for (int i = 0; i < BB_HEIGHT; i++) {
										ri.obst[i].set(x, y + y2 + i);
									}
								}
							}
						}
						y = -10000;
						break;
					}
				}
			}
			y--;
		}
	}
	else {
		ri.sikaiy = ri.length;
	}
	// すべてのマスが視界内かどうかのチェック
	if (ri.squares[ri.length - 1][0] != SquareInfo::Unknown) {
		ri.isallclear = true;
	}
	else {
		ri.isallclear = false;
	}
	// 各マスのゴールからの距離を計算する関数を呼ぶ
	ri.calcdistance();
	return in;
}
