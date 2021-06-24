import argparse
import yaml
import pickle


def main():
    parser = argparse.ArgumentParser(description="Pickleize!")

    parser.add_argument(
        "--input",
        default=None,
        type=str,
        help="Input Yaml file.")
        
    parser.add_argument(
        "--output",
        default=None,
        type=str,
        help="Output Pickle! file.")
        
        
    args = parser.parse_args()
    
    with open(args.input) as labels:
        label_dict = yaml.full_load(labels)
    with open(args.output, 'wb') as pick:       
        pickle.dump(label_dict, pick)

if __name__ == '__main__':
    main()

