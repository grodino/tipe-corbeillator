import json
from datetime import datetime

from path import Path


class Experiment(object):
    """
        Simple tool to manage experiments: save, organize and recall them
        Saving is in json (TODO: and excel-like)
    """

    _base_path: Path
    _dir_path: Path
    _data: dict = {}

    def __init__(self, base_dir, create=False):
        # Instanciate, create or reject base path
        base_path = Path(base_dir)
        
        if create:
            base_path.makedirs_p()
        elif not base_path.isdir() and not create:
            raise ValueError("""
                The base directory given does not exist, set create=True if 
                you want to create it
            """)

        self._base_path = base_path
    
    @classmethod
    def new(cls, base_dir, create=False):
        """
        Create new experiment and the folder associated to it
        """

        experiment = Experiment(base_dir, create=create)

        # List existing experiments
        previous_experiments = experiment._base_path.dirs('experiment#*')
        # Find the last index
        last_index = max(
            [int(name.split('#')[1].split('[')[0]) for name in previous_experiments] \
            + [0]
        )

        # create the folder for this experiment
        time_now = datetime.now()

        experiment._dir_path = Path.joinpath(
            experiment._base_path,
            "experiment#{}[{}_{}_{}][{}h{}]".format(
                int(last_index) + 1,
                time_now.day, 
                time_now.month, 
                time_now.year,
                time_now.hour, 
                time_now.minute
            )
        )
        experiment._dir_path.makedirs()

        return experiment

    @classmethod
    def from_id(cls, id, base_dir):
        """
        Retrieves the experiment data identified by it's id
        """

        experiment = Experiment(base_dir, create=False)
        matching_experiments = experiment._base_path.dirs(
            'experiment#{}*'.format(id)
        )

        if len(matching_experiments) != 1:
            raise ValueError("The experiment id does not exist or too much was found")
        
        experiment._dir_path = matching_experiments[0]
        
        return experiment

    @property
    def data(self):
        """
        If no data, gathers the data from all the files of the 
        experiment else return stored data

        /!\ data json files must have a dict as root object, not a list /!\

        The first layer of keys represent file names
        """

        if self._data:
            return self._data

        self._data = {}

        for file in self._dir_path.files('*.json'):
            with open(file) as file_data:
                self._data[file.name.replace('.json', '')] = json.load(file_data)

        return self._data
    
    def add_data(self, keys, value):
        """
        Add a key and corresponding value to the data[keys[0]][...][keys[n-2]] dict

        Example:
            >>> data = {
            >>>     'foo': {
            >>>         'foofoo': 1,
            >>>         'foobaz': 19
            >>>     }
            >>> }
            >>> keys = ['foo', 'booz']
            >>> value = 42
            >>> add_data(keys, value)
            {
                'foo': {
                    'foofoo': 1,
                    'foobaz': 19,
                    'booz': 42
                }
            }
        """

        if not self._data:
            self.data

        n = len(keys)
        
        if n == 1:
            if not type(value) == dict:
                raise ValueError('The value associated to a root key must be a dict')

            child_dict = self._data
        else:
            child_dict = self._data[keys[0]]

        for i in range(1, n - 1):
            child_dict = child_dict[keys[i]]
        
        if not keys[n-1] in child_dict.keys():
            child_dict[keys[n-1]] = value
        else:
            raise ValueError(
                'The key "{}" you are trying to add already exists'.format(keys[n-1])
                )

    def save(self):
        """
        Over-writes the json files with the new data
        """

        for name in self._data.keys():
            file_path = Path.joinpath(
                self._dir_path,
                '{}.json'.format(name)
            )
            with open(file_path, 'w') as file:
                json.dump(
                    self._data[file_path.name.replace('.json', '')], 
                    file, 
                    indent=4
                )


def main():
    exp = Experiment.new('../data/')
    exp2 = Experiment.from_id(1, '../data/')
    exp2.add_data(['test'], {'a':42})
    print(exp2.data)
    exp2.add_data(['config','mesures', 'effectue', 'acceleration'], {'nb_points': 0, 'fait': 'non'})
    print(exp2.data)
    exp2.save()

if __name__ == '__main__':
    main()