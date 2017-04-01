**ARK State Machine**



**States:**

- Find/ Scan : Quad roams in the arena searching for bots and saving their location.
- Bot Prediction : Predicts the best bot to attack and return its position
- Obstacle Avoidance : Avoids the obstacle
- Strategy : Plans the path and way the bot must be attacked.

**Working of the Machine:**

- Once the take off is successful the control is passed to the FSM. Entry point in the FSM is the Find / Scan state. In find state quad tries to localize itself as well as search for the bot. Detection and localisation is based on probabilistic models including gaussian errors.
- Quad stays in find state until the probability of bot(s)  doesn&#39;t increase a certain threshold , let&#39;s say K%.
- Once quad is certain about the position of bot(s) with probability greater than K% , control is passed to Bot Prediction state which predicts the bot which should be attacked among the bots with certainty higher than K% and sends the location of the bot to Strategy state.
- Strategy state tries to analyze the different states of the bot like position, velocity and direction and accordingly decides how the particular bot should be attacked.
  - Tap on top : Quad taps on the top of the bot to interact
  - Hover in front : Landing the quad in front of bot will be very expensive in terms of time, instead quad will hover at a height low enough to hit the bot in front causing the similar effect as the landing in front would have caused.

If any of the two method fails to execute properly and the probability of the current bot is still high and feasible to attack the state is looped to re-determine the best way among two to again attack the bot, else control simply shifts to bot prediction mode to identify the next best bot to attack.

- Among all the states if quad encounters an obstacle , the state is interrupted and state is switched to obstacle avoidance.







**Rules :**

- State interrupted is re-run after obstacle is avoided excluding the Strategy state where a check redirects it to Strategy or Bot detection state according to the feasibility of attacking the same bot again.
- Anytime the probability of bot(s) go below the K%, state will shift to Find / Scan to increase the certainty of the bot(s). Exceptions :
  - Quad is attacking one bot with high certainty : A threshold delta must be considered in such conditions when the probability of other bots is allowed to fall as low as K-delta % as attacking the bot may increase our points.

- Battery is given the highest priority of all. If the battery is low , state machine will be terminated and the quad will land.

**State Dependence Order :**

Obstacle Avoidance &gt; Find / Scan &gt; Bot Prediction &gt; Strategy