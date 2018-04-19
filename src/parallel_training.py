# resources
import sys
sys.path.append("../src")
from auv_gym_env import make_environment
from ddpg import *
import tensorflow as tf
from multiprocessing import Pool


def main(env_type):

    print("Executing training with " + str(env_type) + " environment.")

    # state and action spaces
    sdim, adim = 12, 2

    # instantiate tensorflow session...
    sess = tf.Session()

    # actor network with -1 to 1 actions bounds
    actor = ActorNetwork(sess, sdim, adim, [1., 1.], 0.0001, 0.001, 64)

    # number of trainable actor variables
    nvar = actor.get_num_trainable_vars()

    # critic network
    critic = CriticNetwork(sess, sdim, adim, 0.001, 0.001, 0.99, nvar)

    # actor noise
    actor_noise = OrnsteinUhlenbeckActionNoise(mu=np.zeros(2))

    # arguments
    args = {
        'actor_lr': 0.001,
        'critic_lr': 0.001,
        'gamma': 0.99,
        'tau': 0.001,
        'buffer_size': 1000000,
        'minibatch_size': 64,
        'random_seed': 1234,
        'max_episodes': 200000,
        'max_episode_len': 100000,
        'render_env': 'store_true',
        'monitor_dir': './results/gym_ddpg',
        'summary_dir': './results/tf_ddpg'
    }

    # instantiate environment
    env = make_environment(env_type, 50, -.1, 5, 5)
    env.render()

    # training
    train(sess, env, args, actor, critic, actor_noise, renf=30)

if __name__ == "__main__":

    # environemnt types
    ets = ['empty', 'fewlarge', 'manysmall']

    # worker pool
    p = Pool(processes=3)

    p.map(main, ets)
