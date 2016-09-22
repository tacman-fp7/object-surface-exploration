"""VAE - Transform a given csv file with iCub taxel values to a csv file with latent representation.

Usage:
  vae.py --taxel-file=<taxelfile> --latent-file=<taxelfile> [--sample_latents]

Options:
  --taxel-file=<taxelfile>          Input csv taxel file.
  --latent-file=<taxelfile>         Output csv latent file.
  --sample_latents                  Sample from latents instead of saving mean and variance.

"""
from docopt import docopt
import numpy as np


def load_parameters():
    return (np.load('mean.npy'),
            np.load('std.npy'),
            [(np.load('W' + str(i) + '.npy'),
              np.load('b' + str(i) + '.npy'))
            for i in range(3)])


def taxel2latent(taxel, parameters):
    n_layer = 3

    mean = parameters[0]
    std = parameters[1]

    taxel = (taxel - mean) / std
    output = taxel
    for i in range(n_layer):
        W = parameters[2][i][0]
        b = parameters[2][i][1]

        output = np.dot(output, W) + b
        if i < n_layer-1:
            output = 1.0 / ( 1.0 + np.exp(-output))

    mean = output[:, :output.shape[1]//2]
    var = output[:, output.shape[1]//2:] ** 2 + 1e-5
    return mean, var


if __name__ == '__main__':
    arguments = docopt(__doc__)
    print arguments

    parameters = load_parameters()

    # load taxel data
    taxel = np.genfromtxt(arguments['--taxel-file'], delimiter=',')

    # transform to latents
    mean, var = taxel2latent(taxel, parameters)
    if arguments['--sample_latents']:
        latents = mean + np.random.randn(*var.shape) * np.sqrt(var)
    else:
        latents = np.concatenate( ( mean, var), 1)

    # save latent data
    np.savetxt(arguments['--latent-file'], latents, delimiter=',')
