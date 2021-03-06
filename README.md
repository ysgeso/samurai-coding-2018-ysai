# samurai-coding-2018-ysai

<a href="https://samuraicoding.info/index-jp.html">samuraicoding 2018</a> 決勝に提出したAIです（チーム名「エーアイエンペラー」）。

2019/3/29 追記。本家のHPに、<a href="https://samuraicoding.info/about-final-racecourses-jp.html">決勝で使用されたレースコースについて</a>というタイトルで説明が書かれました（原因は予想通り、レースコースのパーツを取り込まずに実行したというものでした）。ここで書いた提言とほぼ同じ内容に加え、明らかに挙動がおかしい場合は作者に確認を取るという案も書かれており、満足のいく内容でした。来年度が楽しみです。

2019/3/18 追記。この文章を書いてから気づいたのですが、おそらくコンテストの参加者以外は、決勝で何が起こったかがよく分からないと思いましたので、この文章の最後に決勝で起きたことの考察と提言についてを書きました。AIの説明とは関係ないので、興味がない方は読まなくても大丈夫です。

多くの参加者がはまった、コースのルールの予期せぬ変更のため、本選では自分のAIは途中で動かなくなって敗退しましたが、ここに提出したものは、コメント以外は決勝に提出したもの、そのままです。

決勝でも動くようにするには下記の変更を行えばよいと思います（試してはいませんが・・・）

* raceinfo.hpp の MAX_LENGTH を 200 に、BB_NUM を 32 に変える
* 決勝でのコース予測は無意味だったので、run.shの -cp と -gc オプションを削除する

あと、AI実行時にいろんなパラメータを設定できるようになってますが、デバッグ関連のパラメータは、途中で色々変えているうちにほぼ無意味になってます。特に、記録を取る機能は決勝時に使ったパラメータではほぼ無意味です。

run.shにはかなりたくさんパラメータが指定してありますが、そのうち2つ数字のパラメータが書かれているものは、決勝のAIでは使われていません（ソースコード内にパラメータが指定されています）。

残念ながら決勝ではまともに動きませんでしたが、せっかくなのでやったことの解説も書いてみようと思います（コード自体の可読性はあまり高くないと思いますので）。あと、当日、用事があって参加できず、決勝進出者との交流ができなかったので、誰にも自分がやったことを伝えられずせっかく苦労して作ったAIが埋もれてしまうのがちょっと悲しいので・・・

1．コースアウト判定の高速化

サンプルのコースアウト判定は、サンプルだけあってあまり高速なものとは言えないと思います。コースアウト判定の高速化は、AIの思考速度に大きく影響する部分なので、ここを高速化するのが重要だと思いました。samurai-coding-2017の時もこの部分は工夫しましたが、今回は前回とコースの障害物の仕様が大きく変わったので、一から作る必要がありました。基本的な考え方は以下の通りです。

* マップの障害物を、複数の「幅32 × 長さ8の256ビットのビットマップ」で表現する。
 
* 自分が(x, 0) の座標（０<=x<=19）にいた時に速度(dx, dy) (-5<=dx<=5, -2<=dy<=14)で移動した時の軌跡を表すビットマップをあらかじめ作成しておく（y>8の場合は、2つのビットマップで表現する）。

* 衝突判定は、マップの障害物のビットマップと、移動時の軌跡のビットマップのANDを取って、すべてのビットマップが0でなければ衝突している。

* 軌跡のビットマップはy座標が0の場合だが、任意のy座標のものを作るのは無駄なので、障害物のコースのビットマップを作る際に、y座標を 1~7までそれぞれずらしたものを作り、それと軌跡のビットマップのANDをとればうまくいく。

* 256ビットのANDはSIMD命令を使って高速に行う。

詳しくはコードを見てもらうとして、これでコード内のbenchmark関数を使って計測してみた所、サンプルと比べて約30倍くらい早くなりました。この効果はかなり大きく、おかげでゲーム木の探索時に深さ20の全探索を行う事が可能になりました。

なお、敵と味方の移動の交差判定は、特殊なことはしていません。ネットで見つけた交差判定の式をそのまま使いました。これは、後述のように、敵の行動を3手目までしか考慮しないため、交差判定の計算回数がそれほど多くないと思われたため、頑張って高速化してもほとんと恩恵がなさそうだったからです。

2.ゲーム木について

このゲームは同時着手のゲームですが、交互着手と仮定してmin-maxゲーム木を構築して最も評価値の高い手を選ぶようにしました。ただし、以下のような工夫を行っています。

* 敵のノードは3手目まで構築する。

3手目以降のノードは、敵、味方共に、自分しかゲームに存在しないと仮定して、最も評価値の高い評価値を取る行動を探索する。敵の行動を何手目まで考えるかについては色々試したところ、3手目までが一番強いような感じだったのでそうしました。敵の手をあまり先まで考えても意味がなさそうなゲームなので、3手目くらいが妥当かなと思われます。
  
* ゲーム木の最大深さは20とする。

色々試したところ、20を超えてもほとんど強くならないので、そこで打ち切りました。
 
* 深さ3までの敵のノードの評価値は、以下のような式で計算する。

敵の各行動（最大9種類）に対する評価値を小さい順に並べたものを e1, e2, ..., en とし、aは定数とする。

　(a^(n-1) * e1 + a ^ (n-2) * e2 + ... + en) / (a^(n-1) + a^(n-2) + ... + 1)
  
同時着手ゲームで、単純にmin-max木を使うと、相手はこちら手を見てから最善手を着手するということになるため、自身の手が卑屈な手になりがちなので、それを解消するために相手の最善手以外も考慮するように上記の式を考えてみました。aの値を1より大きくすればするほど、min-max木に近づき、1にすると敵の全評価値の 平均になります。aは後述の自己対戦による学習で決めました。
  
なお、この手法をとったため、αβ枝刈は使えなくなりましたが、敵のノードの深さが３、ゲーム木の深さが20の場合、全探索であるにもかかわらず、ほとんどのコースのゲームで、タイムリミットの30%くらいで余裕をもって思考を終了できました。これは確実に、コースアウト判定の高速化の恩恵だと思われます。
  
3.評価値について

ゲーム木は、敵、味方共に、視界外のマス、又はゴールした場合まで構築し、その位置にたいして下記の式で評価値を計算しました。
  
  各プレーヤーの評価値　= ゴールからの距離による評価値 + 評価値を計算した木の深さによる評価値 + α
  
  最終評価値 = 自分の評価値 - 敵の評価値 * β
  
αは、ゴールした場合などの特殊ケースによる若干の補正です。詳細はコードを見てください。敵の評価値はそのまま使うのではなく、定数βを乗算するようにして色々なβを試してみたところ、強くなる場合が多かったので、βを乗算しています。その値も後述の自己対戦による学習で決めました。βは敵がゴールしている場合とそうでない場合で異なる値を使ってみてはいるのですが、その効果のほどはよくわかりません。
  
ゴールからの距離は、各マスからのマンハッタン距離だとあまり正確ではないので、ダイクストラ法を使って、各マスは、（dx,dy）　（-5<=dx<=5,-5<=dy<=5）の点とつながっているとしてゴールからの距離を計算しました。

木の深さによる評価値ですが、深さに特定の定数（負の値）を乗算した値です。この値も後述の自己対戦による学習で決めました。
  
上記のようにすることで、最も早くゴールするのではなく、最も差をつけてゴールするような行動をとるようにしています。例えば、ゴールが1マスしかない場合で自分が僅差で先行していた場合、わざとゴール前のマスで止まるような行動をとることにより、敵をわざと自分にぶつからせて速度を0にしてゴールの時間差を稼ぐみたいな行動が実際にみられました。
  
4.置換表について

Zobristハッシュを使って各ノードにおける評価値を置換表に記録して起き、既に探索済の状況が現れた時は置換表に記録した値を使う手法をとりました。

5.枝刈について

y速度の最小値を-2,最大値を 8 として枝刈を行いました。9以上にして色々試しても時間がかかるだけでほぼ影響はなかったためそうしました。それ以外の枝刈は特に行っていません。
 
6.コース予測について

ルールには、決勝で使用するコースジェネレータ（予選と同じものを使うと明記されていた・・・）のソースコードが公開されていたことから、これは、コースジェネレーターで生成するコースを考慮に入れてAIを作れというのが出題者からのメッセージだ！と思っていたのですがふたを開けてみると、実際には明らかに予選のコースジェネレータで作られるはずのないコースが決勝で・・・上位者の多くはそれではまってたみたいですね。もちろん私のAIも・・・というわけで実際には無意味（どころか悪影響）だったのですが、せっかくなのでコース予測についても記します。主に3つを実装しました。
 
* パターンによる予測

コースジェネレータのソースコードを読むと、コースに使ういくつかのパターン地形が用意されていました。そのため、パターンの地形と同じ地形の一部が視界の端に見えた場合、その先の地形をそのパターンの地形だと仮定することができます。本AIでは、視界の端の地形がパターンの地形と幅が1マス分でも一致したらパターンの地形と仮定しました。これは確実に予選のコースジェネレータで作られたコースに対しては大きな効果をもたらしました。
  
* トラップ地形の対策

パターンの地形のうちのいくつかは、トラップ地形になっていて、そのパターンの地形の一番奥の地形だけ、ランダムで右端または左端のみ通行可能なようになっていました。トラップ地形の先をすべて見通せないうちに、左右のどちらかに進んでしまうとトラップにはまってしまうというものです。このゲームは、視界を共有するため、先行しているほうが、視界が悪く、トラップ地形にひっかかる可能性が高いのは、先行しているプレーヤーです。そのためトラップ地形はあまり大胆に先に進まないほうが良いケースが多そうでした。
  
そこで、トラップ地形を予測した場合は、右端が空いている場合と左端が空いている場合の両方に対して評価値を計算し、最終的な評価値はその平均をとるという形にしました。そうすることで、トラップ地形に先行した場合は、減速して、場合によっては敵が追い付いて、トラップ地形の先に進むまで待つような行動を取るようになりました。ただ、この行動が本当に最適なのかがよく分からなかったので、最終的には2つの評価値の大きいほうに定数を乗算することにしました。その定数も後述の自己対戦による学習で決めました。
  
* ゴール地形の予測

予選のコースジェネレータでは、必ずスタート地点とゴール地点はISPJまたはSAMURAIのロゴをもじった地形が生成されるようになっていました。スタート地点は必ず視界内なので通常の予測方法で良いのですが、ゴール付近に関しては、視界に入る前からある程度の予測が可能となります。また、コースの幅が10未満の場合は、ロゴの地形の種類が2種類、15未満の場合は3種類、20未満の場合は5種類、20の場合は7種類と、コースの幅が狭い場合は、かなりゴール付近の地形の種類が限定されていました。そこで、ゴール付近を視界に収めた場合は、トラップと同じように、可能性のあるゴールの地形すべてに対して評価値を計算し、その平均を取るという手法をとりました（ただし、幅が20の場合は、可能性が多すぎるので、少し簡略化して計算を行っています）。ゴールにどこまで近づいたらこのゴール予測を行うかについては、定数で定め、その定数も後述の自己対戦による学習で決めました。
  
決勝でスタート地点ではまってたAIはおそらくこれらの地形予測が何らかの悪影響をもたらしたことが原因なんだろうなと思います・・・

7．自己対戦による学習

ここまで、いくつか定数パラメータが出てきましたが、それらの値は下記のような単純な自己対戦による学習で決めました。
 
* 各パラメータの初期値を設定する。
* 各パラメータについて下記の作業を行う。
* 一つのパラメータを一定範囲で少しずつ変化させて最も成績のよい値を見つける
* すべてのパラメータについて上記の作業を行った後、各パラメータについて範囲を狭めて上記の作業を行うということを1~5回行う

敵としては自分のAIでちょっと弱めの設定（コース予測あり、敵の探索深さ2、最大木の深さ10）を設定しました。
 
最初は幅、視界に関係なく同じパラメータでやってたのですが、よく考えると、コースの幅、視界、先手、後手の違いで最適なパラメータが違うのではないかと思って、各コース幅5～20、視界5～20全てに対してコースジェネレーターで100個コースを作って上記の作業を行おうかと考えました。しかし、100コースの対戦だけで5-10分程かかることがわかり、上記の作業１つで3-6時間以上かかることがわかったため、 結局の所は 幅5,10,15,20、視界5,10,20にしぼって繰り返しは1~2回でごまかしました。あと、途中でバグ修正とかして何度も一からやり直すハメに何回もなったり・・・
 
で、そこまでして作ったパラメータ設定はソースコードに書いてある通りですが、見てもらうとわかるとおり、幅、視界が少し違うとかなりバラバラな値になってあまり法則性が見つかりませんでした。まあまあ強いことは確かなのでそのまま作ったパラメータで提出しましたが、100コースくらいでは過学習になってて一般的に強いパラメータは見つけられそうにない感じです。やっぱり学習は計算機資源がないと厳しいですね。

2019/3/18 追記

決勝で何が起こったかの考察

決勝の結果は現時点（2019/3/18時点）ではまだ本家のHPに反映されていませんが、当日の決勝の様子は<a href="https://live.nicovideo.jp/gate/lv317113405">こちらの動画</a>から見ることができます。既にご覧になった方はご存知かと思いますが、決勝進出者の多くのAIが何らかの不具合によってレース開始直後または途中で動作しなくなり、ほぼ不戦勝のようなレースが多発しました（筆者のAIも途中で動かなくなり敗退しました）。この原因について考察します。

なお、この文章はあくまで実際に起きたことについて考察するためのものであり、レースのやり直しなどを要求する意図はありません。

1. 不具合の原因

決勝進出者の多くのAIが同じような不具合に見舞われたことから、原因はAIのプログラムそのもの以外にある可能性が高いと思われます。筆者はコンテスト当日に参加できなかったのですが、参加者のSNSなどを拝見すると、「決勝で使われたコースが、予選のコースと異なっていた」という言及が数多く見受けられました。他のAIについては中身を見ていないのではっきりとは言えませんが、筆者のAIが動かなくなった理由はまさにその点にあります。まず、その点について説明したいと思います。

まず、前提として、<a href="https://samuraicoding.info/final-jp.html">決勝のページ</a>に下記のような記載がされていました。

「決勝ルール：ルールページのゲームルールおよび決勝ルールをご確認ください（ゲームルールは予選と同じです） 」

また、<a href="https://tastasgit.github.io/software-for-SamurAI-Coding-2018-19/documents/final-rule-jp.html">決勝ルール</a>には下記のように記載されていました。</a>

「決勝に用いるレースコースは, コースジェネレータ によって生成する. コースジェネレータは, 予選に用いたものと同一のものを用いる (必要となったバグ修正は除く). 」

上記の文章から、筆者は、決勝で使われるレースは予選と「完全に」同じ条件で作られたレースだと思い込んでしまいました。しかし実際には以下の2点で決勝で使われたコースは予選とは異なっていました。なお、<a href="https://github.com/takashi-chikayama/software-for-SamurAI-Coding-2018-19/tree/master/course_generator">コースジェネレータ―</a>は公開されており、python で書かれたそれほど長くないコードなので、中身をみればどのようなコースが作られるかを知ることはそれほど難しくはありません。

* レースの長さが変わっていた

予選のルールはレースのコースについて下記のように書かれていました。

    コースの長さは50以上100以下である.
    コースの幅は5以上20以下である.
    視界は5以上である.
    考慮時間は200msに制限ステップ数を乗じ1000msを加えたものである.

一方決勝では以下のように書かれていました。

    コースの長さは 100 以上 200 以下である.
    コースの幅は 5 以上 20 以下である.
    視界は 5 以上である.
    考慮時間は 200ms に制限ステップ数を乗じ 1000ms を加えたものである.
    
違いはコースの長さで予選が「50以上100以下」であったのに対して決勝は「100以上200以下」に変化しています。これだけならルールをよく読まなかった方が悪いということになると思いますが、問題はこの部分が「決勝に用いるレースコースは, コースジェネレータ によって生成する. コースジェネレータは, 予選に用いたものと同一のものを用いる (必要となったバグ修正は除く). 」の直後に書かれている点です。筆者のAIが途中で止まった原因はまさにこの点にあり、「同じ」という言葉からコースの長さの条件も予選と同様だと早とちりしてしまい、コースの長さが100を越えると動かなくなるようなプログラムを提出してしまいました。SNSの言及からTaiyoさんも全く同じ原因で動かなかったという発言が見られました。

コースジェネレータは、コースの長さなどを与えて任意の長さのコースを作成できるようになっているのですが、おそらく主催者が言いたかったことは、この「同一のものを用いる」は「コースジェネレータ―」のプログラムに掛かっていて、「コースジェネレータ―」に与えるパラメーターには掛かっていないということだと思うのですが、SNSで参加者の誰かが言及していたように「ただし、コースジェネレータに与える下記のようにパラメータは変化している」を一言書いてもらえればそのような誤解が発生することはなかったのではないかと思われます。

もう一点問題があるとすれば、現在（2019/3/18日時点。決勝の締め切り前までもこの時点と同じ内容が公開されていた）公開されているコースジェネレーターはパラメータを全く与えずにコースを生成させると、コースの長さに関しては長さ50から100までのコースを必ず生成するようになっている点です（コースジェネレータ―の292、293行目の「lmin = 50」「lmax = 100」で定義されている）。この点と、「決勝に用いるレースコースは, コースジェネレータ によって生成する. コースジェネレータは, 予選に用いたものと同一のものを用いる (必要となったバグ修正は除く). 」は矛盾していると言えるのではないかと思うのですが・・・

今回のこの件で個人的に得られた教訓ですが、「ルールは隅々までちゃんと読め！」ってことと、「条件が多少違ってても動くようにプログラムを作るべき！」ってことでした。実際、筆者のAIのコードの定数を2行ほど変えれば、少なくとも次のゴール付近のコース予測で変な動きをする可能性があるとはいえ、途中で止まることなく完走できた可能性が非常に高いと思われるのが残念です。

* 特定の地形が出現しなくなっていた

2点目は、予選で出現した地形パターンが決勝のコースでは一切出現しなかったというものです。コースジェネレーターのソースコードを読めばわかるのですが、コースジェネレータ―で作ったコースはランダムで特定のパターンの地形を生成するようになっていました。このパターンは長さが6～20までの幅があり、その中には「IPSJ]や「SAMURAI」の文字が障害物や水地形で表現されたものもありました。本ゲームは、視界が限られているため、特定のパターンの地形の一部が視界内に入った場合に、視界外の地形をそのパターンの地形だと推測することで実質的に視界を広げることができ、有利な行動を取ることができるようになります。特に長さが16や20のパターンの地形を正しく推測できるかできないかで大幅にAIの強さが変わることは間違いないと断言できます。筆者もこの地形の推測をAIに取り入れましたし、決勝のプレゼンやSNSでそのことに言及していたチームがかなり見受けられていました。なにより、ルールに「公開されているコースジェネレーターを使う」と明記されており、なおかつコースジェネレーターが特定の地形パターンを生成するということから、「この情報をうまく利用しろ！」という主催者側からのメッセージだと受け取った（思い込んだ）AI開発者は多いのではないかと思います。

決勝では、前述したように、予選で出現した地形のパターンが一切登場しなかったのですが、これだけなら「パターンの一部が出てきたら残りの見えてない部分を推測する」というアルゴリズムを使っていた場合、地形パターンが出ないということは、そのアルゴリズムが使われないというだけなのでおそらく決勝進出者のAIに不具合が出る事はなかったと思います。問題はコースジェネレーターが「コースの最初」と「コースの最後」に必ず「ISPJ」または「SAMURAI」のロゴが出現するようなコードになっていた点にあります。スタート地点で動かなくなったAIはおそらく実際にはロゴの地形ではないにも関わらず、「コースの最初はロゴの地形がある」と仮定してAIの行動を決定していたため不具合が起きたと推測されます。あくまで推測ですが「スタート地点の地形をみて一致するロゴの地形を探すが、見つからなくて止まる」、または「実際の地形とは異なるロゴの地形をスタート地点付近の地形と誤認したため、障害物を平地と認識してしまい動けなくなった」あたりではないでしょうか。

あと、ゴール付近で変な動きをしたAIがいくつか見受けられましたが、おそらくゴール付近の地形予測による何らかの影響があったのではないかと思われます。

なお、ジェネレーターのコードは「コースを生成するpythonのコード」と、「コース上に生成する特定の地形パターンデータのファイルを保存したフォルダ」から構成されています。このパターンデータのフォルダを空にした場合、地形パターンのデータはコースに出現しなくなるので、おそらく決勝で使われたコースはパターンデータのフォルダを空にしてコースを生成したと推測されます。従ってルールに記されている「公開されているコースジェネレーターを使う」の「コースジェネレータ」が「コースを生成するpythonのコード」のことだけを表すのであれば、ルールに記載されている通りの方法で決勝のコースも生成したということもできるかもしれませんが、公開されているコースジェネレータ―のコードに地形データのファイルも含まれている以上、「決勝に用いるレースコースは, コースジェネレータ によって生成する. コースジェネレータは, 予選に用いたものと同一のものを用いる (必要となったバグ修正は除く). 」という表記は間違いであったと言えるのではないかと思います。

2. 提言みたいなもの

今回のような事態が起きたのは、決勝のAIの提出時に、提出したAIが正しく動くかどうかを確認するための環境に不備があったためだと思います。

予選では、一度参加者が自分の作成したAIを提出して競い合わせる場が一度設けられており、そこで自分のAIの実力や不具合を試すことができましたが、決勝ではそのような場はありませんでした。参加者の誰かがSNSで言及していましたが、決勝でもそのような場があればそこで決勝のコースが予選と違うことに気づくことができ、今回のような事態にはならなかったのではないかと思います。また、このような対戦型のゲームのAIコンテストでは、自分だけでAIを開発すると、どうしても自分のAIと対戦させることで強化していくという手法になってしまい、アイディアが煮詰まりがちです。AI提出前に他の参加者と対戦する機会を設けることによって自分のAIの欠点や他者のAIのアイディアを取り込むことができ、参加者全体のAIの実力の向上にもつながると思います。他者のAIの動きからアルゴリズムを推測する能力は必要ですし、盗まれたくないアイディアは隠して対戦会には出さないなど、人間くさい駆け引きなども必要となってきますが、それもこのようなコンテストの醍醐味の一つではないでしょうか。

というわけで、2016年度以前のsamuraicodingで行われていたような、AIの締め切りまで常時行われていたような対戦の機会とまでは言いませんが、予選と本選で締め切りまで最低1回（予選は期間が長いので2,3回）は対戦会の機会を設けるのが良いのではないかと思いました。

もう一つ、AIの確認の場として、主催者側が用意した3種類のサンプルAIに対して自身が提出したAIと対戦する場が設けられていました。こちらは常時利用できるもので、自身のAIが動くかどうかを確認できるのですが、一つ重大な欠点がありました。それは対戦で使うコースデータを参加者が用意するというものです。予選の段階で主催者側がコースジェネレータ―でランダムに生成した100のコースデータが用意されていたのですが、決勝用にランダムに生成したコースデータは用意されませんでした。そのため、主催者側が用意した予選用のコースデータを使って決勝提出用のAIの動作確認した参加者の方が多いのではないかと思われますが、これでは決勝の環境での動作のテストにはならず、筆者もここで何度も動作を行って動くと思い込んで提出したAIが決勝で動かないということになりました。

というわけで2つ目の提言ですが、AIの動作確認の場として、提出者が作成したコード以外は、コースデータも含めてすべて主催者側が用意するようにするのが重要だと思います。今回の場合は任意のコースでチェックできるという機能自体は良い機能なので残しておいて欲しいのですが、主催者側が用意した決勝で実際に使われたコースを作成したコースジェネレータで作ったサンプルコースを2,3個選べるようになっていればよかったのではないかと思いました。

以上、やっぱり愚痴みたいなものがちょっと入ってしまいましたが、来年度も時間があれば参加する予定なので、より良いコンテストにするための参考になればと思い僭越ながら提言を書いた次第です。

読んでくれた方はここまでお付き合いいただいてありがとうございました。

もし、質問やコメントなどがあれば、issueのほうに書いてください。
