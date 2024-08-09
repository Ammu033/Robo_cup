from dataclasses import dataclass
import sqlite3
import lxml.html
import markdown
import os

class DatabaseCursor(sqlite3.Cursor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

@dataclass
class ObjectsLocationsDB:
    db_path:str = os.path.join(os.path.dirname(__file__), "objects.db")
    base_info_path:str = "Eindhoven2024"

    def __enter__(self):
        if not os.path.exists(self.db_path):
            self.__connection = sqlite3.connect(self.db_path)
            self.__build_db()
        else:
            os.remove(self.db_path)
            self.__connection = sqlite3.connect(self.db_path)
            self.__build_db()

        return self

    def __exit__(self, type, value, traceback):
        self.__connection.commit()
        self.__connection.close()

    def __build_db(self):
        with self.__connection.cursor(factory = DatabaseCursor) as cursor:
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS `object_classes` (
                `class_id` INTEGER PRIMARY KEY,
                `class_name` VARCHAR(32) NOT NULL
            );""")
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS `objects` (
                `object_id` INTEGER PRIMARY KEY,
                `object_name` VARCHAR(32) NOT NULL,           
                `class_id` INTEGER NOT NULL,   
                FOREIGN KEY (`class_id`) REFERENCES `object_classes`(`class_id`)
            );""")
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS `locations` (
                `location_id` INTEGER PRIMARY KEY,
                `location_name` VARCHAR(32) NOT NULL,
                `placeable` BOOLEAN,
                `class_id` INTEGER NULL,
                FOREIGN KEY (`class_id`) REFERENCES `object_classes`(`class_id`)
            );""")

            self.__parse_objects()
            self.__parse_locations()

    def __parsed_md(self, md_path):
        with open(md_path, "r") as f:
            return lxml.html.fromstring("<html><body>" + markdown.markdown(f.read(), extensions=['tables']) + "</body></html>")
        
    def append_classes(self, category):
        with self.__connection.cursor(factory = DatabaseCursor) as cursor:
            cursor.execute("INSERT INTO `object_classes` (`class_name`) VALUES (?)", (category, ))

    def append_objects(self, object_name, class_name):
        with self.__connection.cursor(factory = DatabaseCursor) as cursor:
            cursor.execute("INSERT INTO `objects` (`object_name`, `class_id`) VALUES (?, (SELECT `class_id` FROM `object_classes` WHERE `class_name` = ?))", (object_name, class_name))

    def append_locations(self, location_name, placeable, class_name):
        with self.__connection.cursor(factory = DatabaseCursor) as cursor:
            if class_name is None:
                cursor.execute("INSERT INTO `locations` (`location_name`, `placeable`) VALUES (?, ?);", (location_name, placeable))
            else:
                cursor.execute("INSERT INTO `locations` (`location_name`, `placeable`, `class_id`) VALUES (?, ?, (SELECT `class_id` FROM `object_classes` WHERE `class_name` = ?));", (location_name, placeable, class_name))

    def __parse_locations(self):
        # renderer = mistune.HTMLRenderer()
        # markdown = mistune.Markdown(renderer, plugins=[table])
        print("\n\n\n")
        md_path = os.path.join(os.path.dirname(__file__), self.base_info_path, "maps", "location_names.md")
        tree = self.__parsed_md(md_path)
        for row in tree.xpath("/html/body/table[1]/tbody")[0].getchildren():
            r = [td.text for td in row.getchildren()]
            if "(p)" in r[1]:
                placeable = True
                location = r[1][:-3].strip().replace("_", " ")
            else:
                placeable = False
                location = r[1].replace("_", " ")

            if r[2] is not None:
                category = r[2].replace("_", " ")
            else:
                category = None

            print(location, placeable, category)
            self.append_locations(location, placeable, category)

    def __parse_objects(self):
        md_path = os.path.join(os.path.dirname(__file__), self.base_info_path, "objects", "objects.md")
        tree = self.__parsed_md(md_path)
        for category, table in zip(tree.xpath("/html/body/h1"), tree.xpath("/html/body/table")):
            tbody = table.getchildren()[1]
            category_name = category.text.split("(")[0][6:].strip().replace("_", " ")
            print(category_name)
            self.append_classes(category_name)
            for tr in tbody.getchildren():
                object_name = tr[0].text.replace("_", " ")
                print(object_name)
                self.append_objects(object_name, category_name)

def generate_database():
    with ObjectsLocationsDB() as db:
        print(db)

if __name__ == "__main__":
    generate_database()            